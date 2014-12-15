#!/usr/bin/env python

# eggbot_sineandlace.py
#
# Generate sinusoidal and "lace" curves.  The curves are described in SVG
# along with the data necessary to regenerate them.  The same data can be
# used to generate new curves which are bounded by a pair of previously
# generated curves.

# Written by Daniel C. Newman for the Eggbot Project
# dan newman @ mtbaldy us
# Last updated 28 November 2010
# 15 October 2010

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

import inkex
import simplepath
import simplestyle
from math import *

VERSION = 1

def parseDesc( str ):

	'''
	Create a dictionary from string description
	'''

	if str is None:
		return {}
	else:
		return dict( [ tok.split( ':' ) for tok in str.split( ';' ) if len( tok ) ] )

def formatDesc( d ):

	'''
	Format an inline name1:value1;name2:value2;... style attribute value
	from a dictionary
	'''

	return ';'.join( [ atr + ':' + str( val ) for atr,val in d.iteritems() ] )

def drawSine( cycles=8, rn=0, rm=0, nPoints=50, offset=[0, 0],
	height=200, width=3200, rescale=0.98, bound1='', bound2='', fun='sine', spline=True ):

	'''
	cycles
	Number of periods to plot within the rectangle of width 'width'

	rn, rm
	Start the function (on the left edge) at the value x = 2 * pi * rn / rm.
	When rm = 0, function is started at x = 0.

	nPoints
	The number of points to sample the function at.  Since the function is
	approximated using Bezier cubic splines, this isn't the number of points
	to plot.

	offset
	(x, y) coordinate of the lower left corner of the bounding rectangle
	in which to plot the function.

	height, width
	The height and width of the rectangle in which to plot the function.
	Ignored when bounding functions, bound1 and bound2, are supplied.

	rescale
	Multiplicative Y-scaling factor by which to rescale the plotted function
	so that it does not fully reach the vertical limits of its bounds.  This
	aids in Eggbot plots by preventing lines from touching and overlapping.

	bound1, bound2
	Descriptions of upper and lower bounding functions by which to limit the
	vertical range of the function to be plotted.

	fun
	May be either 'sine' or 'lace'.
	'''

	'''
	A complicated way of plotting y = sin(x)

	Complicated because we wish to generate the sine wave using a
	parameteric representation.  For plotting a single sine wave in
	Cartesian coordinates, this is overkill.  However, it's useful
	for when we want to compress and expand the amplitude of the
	sine wave in accord with upper and lower boundaries which themselves
	are functions.  By parameterizing everything in sight with the
	same parameter s and restricting s to the range [0, 1], our life
	is made much easier.
	'''

	bounded = False

	if ( not ( bound1 is None ) ) and ( not ( bound2 is None ) ) and \
		( len( bound1 ) > 0 ) and ( len( bound2 ) > 0 ):

		func = parseDesc( bound1 )
		if len( func ) == 0:
			return None, None
		m1 = int( func['rm'] )
		if m1 == 0:
			xMin1 = float( 0 )
		else:
			xMin1 = 2 * pi * float( func['rn'] ) / float( m1 )
		xMax1 = xMin1 + 2 * pi * float( func['cycles'] )
		yMin1 = float( -1 )
		yMax1 = float( +1 )
		yScale1 = float( func['height'] ) / ( yMax1 - yMin1 )
		yOffset1 = float( func['y'] )
		Y1s = lambda s: yOffset1 - yScale1 * sin( xMin1 + ( xMax1 - xMin1 ) * s )

		func = parseDesc( bound2 )
		if len ( func ) == 0:
			return None, None
		m2 = int( func['rm'] )
		if m2 == 0:
			xMin2 = float( 0 )
		else:
			xMin2 = 2 * pi * float( func['rn'] ) / float( m2 )
		xMax2 = xMin2 + 2 * pi * float( func['cycles'] )
		yMin2 = float( -1 )
		yMax2 = float( +1 )
		yScale2 = float( func['height'] ) / ( yMax2 - yMin2 )
		yOffset2 = float( func['y'] )
		Y2s = lambda s: yOffset2 - yScale2 * sin( xMin2 + ( xMax2 - xMin2 ) * s )

		bounded = True

	rescale = float( rescale )
	xOffset = float( offset[0] )
	yOffset = float( offset[1] )

	# Each cycle is 2pi
	n, m = int( 0 ), int( 0 )
	r = 2 * pi * float( cycles )
	if ( int( rm ) == 0 ) or ( int( rn ) == 0 ):
		xMin = float( 0 )
	else:
		xMin = 2 * pi * float( rn ) / float( rm )
	xMax   = xMin + r
	xScale = float( width ) / r		# width / ( xMax - xMin )

	yMin = float( -1 )
	yMax = float( +1 )
	yScale = float( height ) / ( yMax - yMin )

	# Our parametric equations which map the results to our drawing window
	# Note the "-yScale" that's because in SVG, the y-axis runs "backwards"
	if ( fun is None ) or ( fun == '' ):
		fun = 'sine'
	fun = fun.lower()
	if fun == 'sine':
		Xs = lambda s: xOffset + xScale * ( xMax - xMin ) * s
		Ys = lambda s: yOffset - yScale * sin( xMin + ( xMax - xMin ) * s )
		dYdXs = lambda s: -yScale * cos( xMin + ( xMax - xMin ) * s ) / xScale
	elif fun == 'lace':
		Xs = lambda s: xOffset + xScale * ( ( xMax - xMin ) * s + 2 * sin(2 * s * (xMax - xMin) + pi ) )
		dXs = lambda s: xScale * (xMax - xMin ) * ( 1.0 + 4.0 * cos( 2 * s * (xMax - xMin) + pi ) )
		Ys = lambda s: yOffset - yScale * sin( xMin + ( xMax - xMin ) * s )
		dYs = lambda s: -yScale * cos( xMin + ( xMax - xMin ) * s ) * ( xMax - xMin )
		dYdXs = lambda s: dYs( s ) / dXs( s )
	else:
		inkex.errormsg( 'Unknown function %s specifed' % fun )
		return

	# Derivatives: remember the chain rule....
	# dXs = lambda s: xScale * ( xMax - xMin )
	# dYs = lambda s: -yScale * cos( xMin + ( xMax - xMin ) * s ) * ( xMax - xMin )

	# xThird is 1/3 the step size
	nPoints = int( nPoints )

	# xThird is 1/3 the step size; note that Xs(1) - Xs(0) = xScale * ( xMax - xMin )
	xThird = ( Xs( float( 1 ) ) - Xs( float( 0 ) ) ) / float( 3 * (nPoints - 1 ) )

	if bounded:
		yUpper = Y2s( float( 0 ) )
		yLower = Y1s( float( 0 ) )
		yOffset = 0.5 * ( yUpper + yLower )
		yUpper = yOffset + rescale * ( yUpper - yOffset )
		yLower = yOffset + rescale * ( yLower - yOffset )
		yScale = ( yUpper - yLower ) / ( yMax - yMin )

	x1  = Xs( float( 0 ) )
	y1  = Ys( float( 0 ) )
	dx1 = float( 1 )
	dy1 = dYdXs( float( 0 ) )

	# Starting point in the path is ( x, sin(x) )
	pathData = []
	pathData.append(['M ', [ x1 , y1 ] ] )

	for i in range( 1, nPoints ):

		s = float( i ) / float( nPoints - 1 )
		if bounded:
			yUpper = Y2s(s)
			yLower = Y1s(s)
			yOffset = 0.5 * ( yUpper + yLower )
			yUpper = yOffset + rescale * ( yUpper - yOffset )
			yLower = yOffset + rescale * ( yLower - yOffset )
			yScale  = ( yUpper - yLower ) / ( yMax - yMin )

		x2  = Xs( s )
		y2  = Ys( s )
		dx2 = float( 1 )
		dy2 = dYdXs( s )
		if dy2 > float( 10 ):
			dy2 = float( 10 )
		elif dy2 < float( -10 ):
			dy2 = float( -10 )

		# Add another segment to the plot
		if spline:
			pathData.append( [' C ',
					 [ x1 + ( dx1 * xThird ),
					   y1 + ( dy1 * xThird ),
					   x2 - ( dx2 * xThird ),
					   y2 - ( dy2 * xThird ),
					   x2, y2 ] ] )
		else:
			pathData.append([' L ', [ x1, y1 ] ] )
			pathData.append([' L ', [ x2, y2 ] ] )
		x1  = x2
		y1  = y2
		dx1 = dx2
		dy1 = dy2

	pathDesc = \
		'version:%d;style:linear;function:sin(x);' % VERSION + \
		'cycles:%f;rn:%d;rm:%d;points:%d' % ( cycles, rn, rm, nPoints ) + \
		';width:%d;height:%d;x:%d;y:%d' % ( width, height, offset[0], offset[1] )

	return pathData, pathDesc

class SpiroSine( inkex.Effect ):

	nsURI    = 'http://sample.com/ns'
	nsPrefix = 'doof'

	def __init__(self):

		inkex.Effect.__init__(self)

		self.OptionParser.add_option( "--tab",	#NOTE: value is not used.
			action="store", type="string",
			dest="tab", default="splash",
			help="The active tab when Apply was pressed" )

		self.OptionParser.add_option('--fCycles', dest='fCycles',
			type='float', default=float( 10 ), action='store',
			help='Number of cycles (periods)' )

		self.OptionParser.add_option('--nrN', dest='nrN',
			type='int', default=int( 0 ), action='store',
			help='Start x at 2 * pi * n / m' )

		self.OptionParser.add_option('--nrM', dest='nrM',
			type='int', default=int( 0 ), action='store',
			help='Start x at 2 * pi * n / m' )

		self.OptionParser.add_option('--fRecess', dest='fRecess',
			type='float', default=float( 2 ), action='store',
			help='Recede from envelope by factor' )

		self.OptionParser.add_option("--nSamples", dest="nSamples",
			type="int", default=float( 50 ), action="store",
			help="Number of points to sample" )

		self.OptionParser.add_option("--nWidth", dest="nWidth",
			type="int", default=int( 3200 ), action="store",
			help="Width in pixels" )

		self.OptionParser.add_option("--nHeight", dest="nHeight",
			type="int", default=int( 100 ), action="store",
			help="Height in pixels" )

		self.OptionParser.add_option("--nOffsetX", dest="nOffsetX",
			type="int", default=int( 0 ), action="store",
			help="Starting x coordinate (pixels)" )

		self.OptionParser.add_option("--nOffsetY", dest="nOffsetY",
			type="int", default=int( 400 ), action="store",
			help="Starting y coordinate (pixels)" )

		self.OptionParser.add_option('--bLace', dest='bLace',
			type='inkbool', default=False, action='store',
			help='Lace' )

		self.OptionParser.add_option('--bSpline', dest='bSpline',
			type='inkbool', default=True, action='store',
			help='Spline' )

		self.recess = float( 0.95 )

	def effect( self ):

		inkex.NSS[self.nsPrefix] = self.nsURI

		if self.options.bLace:
			func = 'lace'
		else:
			func = 'sine'

		fRecess = float( 1 )
		if self.options.fRecess > 0.0:
			fRecess = 1.0 - self.options.fRecess / float( 100 )
			if fRecess <= 0.0:
				fRecess = float( 0 )

		if self.options.ids:
			if len( self.options.ids ) == 2:
				attr = self.selected[self.options.ids[0]].attrib
				desc1 = self.selected[self.options.ids[0]].get( inkex.addNS( 'desc', self.nsPrefix ) )
				desc2 = self.selected[self.options.ids[1]].get( inkex.addNS( 'desc', self.nsPrefix ) )
				if ( not desc1 ) or ( not desc2 ):
					inkex.errormsg( 'Selected objects do not smell right' )
					return
				path_data, path_desc = drawSine( self.options.fCycles,
					self.options.nrN,
					self.options.nrM,
					self.options.nSamples,
					[ self.options.nOffsetX, self.options.nOffsetY ],
					self.options.nHeight,
					self.options.nWidth,
					fRecess,
					desc1, desc2, func, self.options.bSpline )
			else:
				inkex.errormsg( 'Exactly two objects must be selected' )
				return
		else:
			self.document.getroot().set( inkex.addNS( self.nsPrefix, 'xmlns' ), self.nsURI )

			path_data, path_desc = drawSine( self.options.fCycles,
				self.options.nrN,
				self.options.nrM,
				self.options.nSamples,
				[ self.options.nOffsetX, self.options.nOffsetY ],
				self.options.nHeight,
				self.options.nWidth,
				fRecess,
				None, None, func, self.options.bSpline )

		style = { 'stroke': 'black', 'stroke-width': '1', 'fill': 'none' }
		path_attrs = {
			'style': simplestyle.formatStyle( style ),
			'd': simplepath.formatPath( path_data ),
			inkex.addNS( 'desc', self.nsPrefix ): path_desc }
		newpath = inkex.etree.SubElement( self.document.getroot(),
			inkex.addNS( 'path', 'svg '), path_attrs, nsmap=inkex.NSS )

if __name__ == '__main__':

	e = SpiroSine()
	e.affect()
