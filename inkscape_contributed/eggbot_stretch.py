#!/usr/bin/env python

# Written by Daniel C. Newman ( dan dot newman at mtbaldy dot us )
# 19 October 2010

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

import math
import inkex
import simplepath
import simplestyle
import simpletransform
import cubicsuperpath
import cspsubdiv
import bezmisc

N_PAGE_WIDTH = 3200
N_PAGE_HEIGHT = 800

def inverseTransform ( tran ):
	'''
	An SVG transform matrix looks like

		[  a   c   e  ]
		[  b   d   f  ]
		[  0   0   1  ]

	And it's inverse is

		[  d   -c   cf - de  ]
		[ -b    a   be - af  ] * ( ad - bc ) ** -1
		[  0    0      1     ]

	And, no reasonable 2d coordinate transform will have
	the products ad and bc equal.

	SVG represents the transform matrix column by column as
	matrix(a b c d e f) while Inkscape extensions store the
	transform matrix as

		[[a, c, e], [b, d, f]]

	To invert the transform stored Inskcape style, we wish to
	produce

		[[d/D, -c/D, (cf - de)/D], [-b/D, a/D, (be-af)/D]]

	where

		D = 1 / (ad - bc)
	'''

	D = tran[0][0] * tran[1][1] - tran[1][0] * tran[0][1]
	if D == 0:
		return None

	return [[tran[1][1]/D, -tran[0][1]/D,
			(tran[0][1]*tran[1][2] - tran[1][1]*tran[0][2])/D],
			[-tran[1][0]/D, tran[0][0]/D,
			(tran[1][0]*tran[0][2] - tran[0][0]*tran[1][2])/D]]

def parseLengthWithUnits( str ):

	'''
	Parse an SVG value which may or may not have units attached
	This version is greatly simplified in that it only allows: no units,
	units of px, and units of %.  Everything else, it returns None for.
	There is a more general routine to consider in scour.py if more
	generality is ever needed.
	'''

	u = 'px'
	s = str.strip()
	if s[-2:] == 'px':
		s = s[:-2]
	elif s[-1:] == '%':
		u = '%'
		s = s[:-1]

	try:
		v = float( s )
	except:
		return None, None

	return v, u

def subdivideCubicPath( sp, flat, i=1 ):

	'''
	[ Lifted from eggbot.py with impunity ]

	Break up a bezier curve into smaller curves, each of which
	is approximately a straight line within a given tolerance
	(the "smoothness" defined by [flat]).

	This is a modified version of cspsubdiv.cspsubdiv(): rewritten
	because recursion-depth errors on complicated line segments
	could occur with cspsubdiv.cspsubdiv().
	'''

	while True:
		while True:
			if i >= len( sp ):
				return

			p0 = sp[i - 1][1]
			p1 = sp[i - 1][2]
			p2 = sp[i][0]
			p3 = sp[i][1]

			b = ( p0, p1, p2, p3 )

			if cspsubdiv.maxdist( b ) > flat:
				break

			i += 1

		one, two = bezmisc.beziersplitatt( b, 0.5 )
		sp[i - 1][2] = one[1]
		sp[i][0] = two[2]
		p = [one[2], one[3], two[1]]
		sp[i:1] = [p]

class Map( inkex.Effect ):

	def __init__( self ):

		inkex.Effect.__init__( self )

		self.OptionParser.add_option('--smoothness', dest='smoothness',
			type='float', default=float( 0.2 ), action='store',
			help='Curve smoothing (less for more)' )

		self.OptionParser.add_option('--maxDy', dest='maxDy',
			type='float', default=float( 5.0 ), action='store',
			help='Vertical smoothing (less for more)' )

		self.cx = float( N_PAGE_WIDTH ) / 2.0
		self.cy = float( N_PAGE_HEIGHT ) / 2.0
		self.xmin, self.xmax = ( 1.0E70, -1.0E70 )
		self.maxDy = float( 5 )
		self.paths = {}
		self.transforms = {}

		# For handling an SVG viewbox attribute, we will need to know the
		# values of the document's <svg> width and height attributes as well
		# as establishing a transform from the viewbox to the display.

		self.docWidth = float( N_PAGE_WIDTH )
		self.docHeight = float( N_PAGE_HEIGHT )
		self.docTransform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]

	def getLength( self, name, default ):

		'''
		Get the <svg> attribute with name "name" and default value "default"
		Parse the attribute into a value and associated units.  Then, accept
		no units (''), units of pixels ('px'), and units of percentage ('%').
		'''

		str = self.document.getroot().get( name )
		if str:
			v, u = parseLengthWithUnits( str )
			if not v:
				# Couldn't parse the value
				return None
			elif ( u == '' ) or ( u == 'px' ):
				return v
			elif u == '%':
				return float( default ) * v / 100.0
			else:
				# Unsupported units
				return None
		else:
			# No width specified; assume the default value
			return float( default )

	def getDocProps( self ):

		'''
		Get the document's height and width attributes from the <svg> tag.
		Use a default value in case the property is not present or is
		expressed in units of percentages.
		'''

		self.docHeight = self.getLength( 'height', N_PAGE_HEIGHT )
		self.docWidth = self.getLength( 'width', N_PAGE_WIDTH )
		if ( self.docHeight == None ) or ( self.docWidth == None ):
			return False
		else:
			return True

	def handleViewBox( self ):

		'''
		Set up the document-wide transform in the event that the document has an SVG viewbox
		'''

		if self.getDocProps():
			viewbox = self.document.getroot().get( 'viewBox' )
			if viewbox:
				vinfo = viewbox.strip().replace( ',', ' ' ).split( ' ' )
				if ( vinfo[2] != 0 ) and ( vinfo[3] != 0 ):
					sx = self.docWidth / float( vinfo[2] )
					sy = self.docHeight / float( vinfo[3] )
					self.docTransform = simpletransform.parseTransform( 'scale(%f,%f)' % (sx, sy) )

	def getPathVertices( self, path, node=None, transform=None, find_bbox=False ):

		'''
		Decompose the path data from an SVG element into individual
		subpaths, each subpath consisting of absolute move to and line
		to coordinates.  Place these coordinates into a list of polygon
		vertices.
		'''

		if ( not path ) or ( len( path ) == 0 ):
			# Nothing to do
			return None

		# parsePath() may raise an exception.  This is okay
		sp = simplepath.parsePath( path )
		if ( not sp ) or ( len( sp ) == 0 ):
			# Path must have been devoid of any real content
			return None

		# Get a cubic super path
		p = cubicsuperpath.CubicSuperPath( sp )
		if ( not p ) or ( len( p ) == 0 ):
			# Probably never happens, but...
			return None

		if transform:
			simpletransform.applyTransformToPath( transform, p )

		# Now traverse the cubic super path
		subpath_list = []
		subpath_vertices = []
		for sp in p:
			if len( subpath_vertices ):
				subpath_list.append( subpath_vertices )
			subpath_vertices = []
			last_csp = None
			subdivideCubicPath( sp, float( self.options.smoothness ) )
			for csp in sp:
				if ( last_csp != None ) and ( math.fabs( csp[1][1] - last_csp[1] ) > self.options.maxDy ):
					dy = ( csp[1][1] - last_csp[1] )
					dx = ( csp[1][0] - last_csp[0] )
					nsteps = math.ceil( math.fabs( dy / self.options.maxDy ) )
					for n in range( 1, int( 1 + nsteps ) ):
						s = n / nsteps
						subpath_vertices.append( [ last_csp[0] + s * dx, last_csp[1] + s * dy ] )
				else:
					# Add this vertex to the list of vetices
					subpath_vertices.append( csp[1] )
				last_csp = csp[1]
				if find_bbox:
					if last_csp[0] < self.xmin:
						self.xmin = last_csp[0]
					elif last_csp[0] > self.xmax:
						self.xmax = last_csp[0]

		# Handle final subpath
		if len( subpath_vertices ):
			subpath_list.append( subpath_vertices )

		if len( subpath_list ) > 0:
			self.paths[node] = subpath_list
			self.transforms[node] = transform

	def mapPathVertices( self, node ):

		steps2rads = math.pi / float( 1600 )

		transform = self.transforms[node]
		if transform is None:
			invTransform = None
		else:
			invTransform = inverseTransform( transform )

		newPath = ''
		for subpath in self.paths[node]:
			lastPoint = subpath[0]
			lastPoint[0] = self.cx + ( lastPoint[0] - self.cx ) / math.cos( ( lastPoint[1] - self.cy ) * steps2rads )
			if invTransform != None:
				simpletransform.applyTransformToPoint( invTransform, lastPoint )
			newPath += ' M %f,%f' % ( lastPoint[0], lastPoint[1] )
			for point in subpath[1:]:
				x = self.cx + ( point[0] - self.cx ) / math.cos( ( point[1] - self.cy ) * steps2rads )
				pt = [x, point[1] ]
				if invTransform != None:
					simpletransform.applyTransformToPoint( invTransform, pt )
				newPath += ' l %f,%f' % ( pt[0] - lastPoint[0], pt[1] - lastPoint[1] )
				lastPoint = pt

		self.paths[node] = newPath

	def recursivelyTraverseSvg( self, aNodeList,
		matCurrent=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
		parent_visibility='visible', find_bbox=False ):

		'''
		[ This too is largely lifted from eggbot.py ]

		Recursively walk the SVG document, building polygon vertex lists
		for each graphical element we support.

		Rendered SVG elements:
			<circle>, <ellipse>, <line>, <path>, <polygon>, <polyline>, <rect>

		Supported SVG elements:
			<group>, <use>

		Ignored SVG elements:
			<defs>, <eggbot>, <metadata>, <namedview>, <pattern>,
			processing directives

		All other SVG elements trigger an error (including <text>)
		'''

		for node in aNodeList:

			# Ignore invisible nodes
			v = node.get( 'visibility', parent_visibility )
			if v == 'inherit':
				v = parent_visibility
			if v == 'hidden' or v == 'collapse':
				pass

			# First apply the current matrix transform to this node's tranform
			matNew = simpletransform.composeTransform( matCurrent, simpletransform.parseTransform( node.get( "transform" ) ) )

			if node.tag == inkex.addNS( 'g', 'svg' ) or node.tag == 'g':

				self.recursivelyTraverseSvg( node, matNew, v, find_bbox )

			elif node.tag == inkex.addNS( 'use', 'svg' ) or node.tag == 'use':

				# A <use> element refers to another SVG element via an xlink:href="#blah"
				# attribute.  We will handle the element by doing an XPath search through
				# the document, looking for the element with the matching id="blah"
				# attribute.  We then recursively process that element after applying
				# any necessary (x,y) translation.
				#
				# Notes:
				#  1. We ignore the height and width attributes as they do not apply to
				#     path-like elements, and
				#  2. Even if the use element has visibility="hidden", SVG still calls
				#     for processing the referenced element.  The referenced element is
				#     hidden only if its visibility is "inherit" or "hidden".

				refid = node.get( inkex.addNS( 'href', 'xlink' ) )
				if not refid:
					pass

				# [1:] to ignore leading '#' in reference
				path = '//*[@id="%s"]' % refid[1:]
				refnode = node.xpath( path )
				if refnode:
					x = float( node.get( 'x', '0' ) )
					y = float( node.get( 'y', '0' ) )
					# Note: the transform has already been applied
					if ( x != 0 ) or (y != 0 ):
					       	matNew2 = composeTransform( matNew, parseTransform( 'translate(%f,%f)' % (x,y) ) )
					else:
					       	matNew2 = matNew
					v = node.get( 'visibility', v )
					self.recursivelyTraverseSvg( refnode, matNew2, v, find_bbox )

			elif node.tag == inkex.addNS( 'path', 'svg' ):

				path_data = node.get( 'd')
				if path_data:
					self.getPathVertices( path_data, node, matNew, find_bbox )

			elif node.tag == inkex.addNS( 'rect', 'svg' ) or node.tag == 'rect':

				# Manually transform
				#
				#    <rect x="X" y="Y" width="W" height="H"/>
				#
				# into
				#
				#    <path d="MX,Y lW,0 l0,H l-W,0 z"/>
				#
				# I.e., explicitly draw three sides of the rectangle and the
				# fourth side implicitly

				# Create a path with the outline of the rectangle
				x = float( node.get( 'x' ) )
				y = float( node.get( 'y' ) )
				if ( not x ) or ( not y ):
					pass
				w = float( node.get( 'width', '0' ) )
				h = float( node.get( 'height', '0' ) )
				a = []
				a.append( ['M ', [x, y]] )
				a.append( [' l ', [w, 0]] )
				a.append( [' l ', [0, h]] )
				a.append( [' l ', [-w, 0]] )
				a.append( [' Z', []] )
				self.getPathVertices( simplepath.formatPath( a ), node, matNew, find_bbox )

			elif node.tag == inkex.addNS( 'line', 'svg' ) or node.tag == 'line':

				# Convert
				#
				#   <line x1="X1" y1="Y1" x2="X2" y2="Y2/>
				#
				# to
				#
				#   <path d="MX1,Y1 LX2,Y2"/>

				x1 = float( node.get( 'x1' ) )
				y1 = float( node.get( 'y1' ) )
				x2 = float( node.get( 'x2' ) )
				y2 = float( node.get( 'y2' ) )
				if ( not x1 ) or ( not y1 ) or ( not x2 ) or ( not y2 ):
					pass
				a = []
				a.append( ['M ', [x1, y1]] )
				a.append( [' L ', [x2, y2]] )
				self.getPathVertices( simplepath.formatPath( a ), node, matNew, find_bbox )

			elif node.tag == inkex.addNS( 'polyline', 'svg' ) or node.tag == 'polyline':

				# Convert
				#
				#  <polyline points="x1,y1 x2,y2 x3,y3 [...]"/>
				#
				# to
				#
				#   <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...]"/>
				#
				# Note: we ignore polylines with no points

				pl = node.get( 'points', '' ).strip()
				if pl == '':
					pass

				pa = pl.split()
				d = "".join( ["M " + pa[i] if i == 0 else " L " + pa[i] for i in range( 0, len( pa ) )] )
				self.getPathVertices( d, node, matNew, find_bbox )

			elif node.tag == inkex.addNS( 'polygon', 'svg' ) or node.tag == 'polygon':

				# Convert
				#
				#  <polygon points="x1,y1 x2,y2 x3,y3 [...]"/>
				#
				# to
				#
				#   <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...] Z"/>
				#
				# Note: we ignore polygons with no points

				pl = node.get( 'points', '' ).strip()
				if pl == '':
					pass

				pa = pl.split()
				d = "".join( ["M " + pa[i] if i == 0 else " L " + pa[i] for i in range( 0, len( pa ) )] )
				d += " Z"
				self.getPathVertices( d, node, matNew, find_bbox )

			elif node.tag == inkex.addNS( 'ellipse', 'svg' ) or \
				node.tag == 'ellipse' or \
				node.tag == inkex.addNS( 'circle', 'svg' ) or \
				node.tag == 'circle':

					# Convert circles and ellipses to a path with two 180 degree arcs.
					# In general (an ellipse), we convert
					#
					#   <ellipse rx="RX" ry="RY" cx="X" cy="Y"/>
					#
					# to
					#
					#   <path d="MX1,CY A RX,RY 0 1 0 X2,CY A RX,RY 0 1 0 X1,CY"/>
					#
					# where
					#
					#   X1 = CX - RX
					#   X2 = CX + RX
					#
					# Note: ellipses or circles with a radius attribute of value 0 are ignored

					if node.tag == inkex.addNS( 'ellipse', 'svg' ) or node.tag == 'ellipse':
						rx = float( node.get( 'rx', '0' ) )
						ry = float( node.get( 'ry', '0' ) )
					else:
						rx = float( node.get( 'r', '0' ) )
						ry = rx
					if rx == 0 or ry == 0:
						pass

					cx = float( node.get( 'cx', '0' ) )
					cy = float( node.get( 'cy', '0' ) )
					x1 = cx - rx
					x2 = cx + rx
					d = 'M %f,%f ' % ( x1, cy ) + \
						'A %f,%f ' % ( rx, ry ) + \
						'0 1 0 %f,%f ' % ( x2, cy ) + \
						'A %f,%f ' % ( rx, ry ) + \
						'0 1 0 %f,%f' % ( x1, cy )
					self.mapPathVertices( d, node, matNew, find_bbox )

			elif node.tag == inkex.addNS( 'pattern', 'svg' ) or node.tag == 'pattern':

				pass

			elif node.tag == inkex.addNS( 'metadata', 'svg' ) or node.tag == 'metadata':

				pass

			elif node.tag == inkex.addNS( 'defs', 'svg' ) or node.tag == 'defs':

				pass

			elif node.tag == inkex.addNS( 'namedview', 'sodipodi' ) or node.tag == 'namedview':

				pass

			elif node.tag == inkex.addNS( 'eggbot', 'svg' ) or node.tag == 'eggbot':

				pass

			elif node.tag == inkex.addNS( 'text', 'svg' ) or node.tag == 'text':

				inkex.errormsg( 'Warning: unable to draw text, please convert it to a path first.' )

				pass

			elif not isinstance( node.tag, basestring ):

				pass

			else:

				inkex.errormsg( 'Warning: unable to draw object <%s>, please convert it to a path first.' % node.tag )
				pass

	def recursivelyReplaceSvg( self, nodes, parent_visibility='visible' ):

		for i in range( 0, len( nodes ) ):

			node = nodes[i]

			# Ignore invisible nodes
			v = node.get( 'visibility', parent_visibility )
			if v == 'inherit':
				v = parent_visibility
			if v == 'hidden' or v == 'collapse':
				pass

			if node.tag == inkex.addNS( 'g', 'svg' ) or node.tag == 'g':

				self.recursivelyReplaceSvg( node, parent_visibility=v )

			elif node.tag == inkex.addNS( 'path', 'svg' ):

				if self.paths.has_key( node ):
					# Change the path data to be the new path
					node.set( 'd', self.paths[node][1:] )
					del self.paths[node]

			elif node.tag == inkex.addNS( 'use', 'svg' ) or node.tag == 'use' or \
				node.tag == inkex.addNS( 'rect', 'svg' ) or node.tag == 'rect' or \
				node.tag == inkex.addNS( 'line', 'svg' ) or node.tag == 'line' or \
				node.tag == inkex.addNS( 'polyline', 'svg' ) or node.tag == 'polyline' or \
				node.tag == inkex.addNS( 'polygon', 'svg' ) or node.tag == 'polygon' or \
				node.tag == inkex.addNS( 'ellipse', 'svg' ) or node.tag == 'ellipse' or \
				node.tag == inkex.addNS( 'circle', 'svg' ) or node.tag == 'circle':
				# Replace this element with a <path> element

				if self.paths.has_key( node ):
					# Create a new <path> element
					# We simply copy all of the attributes from
					# the old element to this new element even though
					# some of the attributes are no longer relevant
					newNode = inkex.etree.Element( inkex.addNS( 'path', 'svg' ), node.attrib )
					newNode.set( 'd', self.paths[node][1:] )

					# Now replace the old element with this element
					nodes[i] = newNode

					# And dispose of the old data and element
					del self.paths[node]
					del node

			else:

				pass

	def recursivelyGetEnclosingTransform( self, node ):

		'''
		Determine the cumulative transform which node inherits from
		its chain of ancestors.
		'''
		node = node.getparent()
		if node is not None:
			parent_transform = self.recursivelyGetEnclosingTransform( node )
			node_transform = node.get( 'transform', None )
			if node_transform is None:
				return parent_transform
			else:
				tr = simpletransform.parseTransform( node_transform )
				if parent_transform is None:
					return tr
				else:
					return simpletransform.composeTransform( parent_transform, tr )
		else:
			return self.docTransform

	def effect( self ):

		# Viewbox handling
		self.handleViewBox()

		# Locate the center of the document by obtaining its dimensions
		if ( self.docHeight is None ) or (self.docWidth is None ):
			inkex.errormsg( 'Document has inappropriate width or height units' )
			return
		self.cy = self.docHeight / float ( 2 )
		self.cx = self.docWidth / float( 2 )

		# First traverse the document (or selected items), reducing
		# everything to line segments.  If working on a selection,
		# then determine the selection's bounding box in the process.
		# (Actually, we just need to know it's extrema on the x-axis.)

		if self.options.ids:
			# Traverse the selected objects
			for id in self.options.ids:
				transform = self.recursivelyGetEnclosingTransform( self.selected[id] )
				self.recursivelyTraverseSvg( [self.selected[id]], transform, find_bbox=True )
			# Use as the vertical centerline the midpoint between
			# the bounding box's extremal X coordinates
			self.cx = 0.5 * ( self.xmin + self.xmax )
		else:
			# Traverse the entire document building new, transformed paths
			self.recursivelyTraverseSvg( self.document.getroot(), self.docTransform )

		# Now that we know the x-axis extrema, we can remap the data
		# Had we know the x-axis extrema in advance (i.e., operating
		# on the entire document), then we could have done the mapping
		# at the same time we "rendered" everything to line segments.

		for key in self.paths:
			self.mapPathVertices( key )

		# And now replace the old paths with the new paths
		# WE DO NOT compute and replace the paths in the same pass!
		# So doing can cause multiple transformations of cloned paths

		self.recursivelyReplaceSvg( self.document.getroot(), self.docTransform )

if __name__ == '__main__':

	e = Map()
	e.affect()
