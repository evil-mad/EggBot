#!/usr/bin/env python

# twist.py -- Primarily a simple example of writing an Inkscape extension
#             which manipulates objects in a drawing.
#
# For a polygon with vertices V[0], V[1], V[2], ..., V[n-1] iteratively
# move each vertex V[i] by a constant factor 0 < s < 1.0 along the edge
# between V[i] and V[i+1 modulo n] for 0 <= i <= n-1.
#
# This extension operates on every selected closed path, or, if no paths
# are selected, then every closed path in the document.  Since the "twisting"
# effect only concerns itself with individual paths, no effort is made to
# worry about the transforms applied to the paths.  That is, it is not
# necessary to worry about tracking SVG transforms as all the work can be
# done using the untransformed coordinates of each path.

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

import inkex
import simplepath
import simplestyle
import simpletransform
import cubicsuperpath
import cspsubdiv
import bezmisc

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

def distanceSquared( P1, P2 ):

	'''
	Pythagorean distance formula WITHOUT the square root.  Since
	we just want to know if the distance is less than some fixed
	fudge factor, we can just square the fudge factor once and run
	with it rather than compute square roots over and over.
	'''

	dx = P2[0] - P1[0]
	dy = P2[1] - P1[1]

	return ( dx * dx + dy * dy )

class Twist( inkex.Effect ):

	def __init__( self ):

		inkex.Effect.__init__( self )
		self.OptionParser.add_option(
			"--nSteps", action="store", type="int",
			dest="nSteps", default=8,
			help="Number of iterations to take" )
		self.OptionParser.add_option(
			"--fRatio", action="store", type="float",
			dest="fRatio", default=float( 0.2 ),
			help="Some ratio" )

		'''
		Store each path in an associative array (dictionary) indexed
		by the lxml.etree pointer for the SVG document element
		containing the path.  Looking up the path in the dictionary
		yields a list of lists.  Each of these lists is a subpath
		# of the path.  E.g., for the SVG path

			<path d="M 10,10 l 0,5 l 5,0 l 0,-5 Z M 30,30 L 30,60"/>

		 we'd have two subpaths which will be reduced to absolute
		 coordinates.

			subpath_1 = [ [10, 10], [10, 15], [15, 15], [15, 10], [10,10] ]
			subpath_2 = [ [30, 30], [30, 60] ]
			self.paths[<node pointer>] = [ subpath_1, subpath_2 ]

		All of the paths and their subpaths could be drawn as follows:

			for path in self.paths:
				for subpath in self.paths[path]:
					first = True
					for vertex in subpath:
						if first:
							moveto( vertex[0], vertex[1] )
							first = False
						else:
							lineto( vertex[0], vertex[1] )

		NOTE: drawing all the paths like the above would not in general
		give the correct rendering of the document UNLESS path transforms
		were also tracked and applied.
		'''

		self.paths = {}
		self.paths_clone_transform = {}

	def addPathVertices( self, path, node=None, transform=None, cloneTransform=None ):

		'''
		Decompose the path data from an SVG element into individual
		subpaths, each subpath consisting of absolute move to and line
		to coordinates.  Place these coordinates into a list of polygon
		vertices.
		'''

		if ( not path ) or ( len( path ) == 0 ):
			# Nothing to do
			return

		# parsePath() may raise an exception.  This is okay
		sp = simplepath.parsePath( path )
		if ( not sp ) or ( len( sp ) == 0 ):
			# Path must have been devoid of any real content
			return

		# Get a cubic super path
		p = cubicsuperpath.CubicSuperPath( sp )
		if ( not p ) or ( len( p ) == 0 ):
			# Probably never happens, but...
			return

		#if transform:
		#	simpletransform.applyTransformToPath( transform, p )

		# Now traverse the cubic super path
		subpath_list = []
		subpath_vertices = []
		for sp in p:
			if len( subpath_vertices ):
				# There's a prior subpath: see if it is closed and should be saved
				if distanceSquared( subpath_vertices[0], subpath_vertices[-1] ) < 1:
					# Keep the prior subpath: it appears to be a closed path
					subpath_list.append( subpath_vertices )
			subpath_vertices = []
			subdivideCubicPath( sp, float( 0.2 ) )
			for csp in sp:
				# Add this vertex to the list of vetices
				subpath_vertices.append( csp[1] )

		# Handle final subpath
		if len( subpath_vertices ):
			if distanceSquared( subpath_vertices[0], subpath_vertices[-1] ) < 1:
				# Path appears to be closed so let's keep it
				subpath_list.append( subpath_vertices )

		# Empty path?
		if len( subpath_list ) == 0:
			return

		# Store the list of subpaths in a dictionary keyed off of the path's node pointer
		self.paths[node] = subpath_list
		self.paths_clone_transform[node] = cloneTransform

	def recursivelyTraverseSvg( self, aNodeList,
		matCurrent=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
		parent_visibility='visible', cloneTransform=None ):

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
			matNew = simpletransform.composeTransform( matCurrent,
				simpletransform.parseTransform( node.get( "transform" ) ) )

			if node.tag == inkex.addNS( 'g', 'svg' ) or node.tag == 'g':

				self.recursivelyTraverseSvg( node, matNew, parent_visibility=v )

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
					self.recursivelyTraverseSvg( refnode, matNew2,
						parent_visibility=v, cloneTransform=node.get( 'transform' ) )

			elif node.tag == inkex.addNS( 'path', 'svg' ):

				path_data = node.get( 'd')
				if path_data:
					self.addPathVertices( path_data, node, matNew, cloneTransform )

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
				self.addPathVertices( simplepath.formatPath( a ), node, matNew, cloneTransform )

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
				self.addPathVertices( simplepath.formatPath( a ), node, matNew, cloneTransform )

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
				self.addPathVertices( d, node, matNew, cloneTransform )

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
				self.addPathVertices( d, node, matNew, cloneTransform )

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
					self.addPathVertices( d, node, matNew, cloneTransform )

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

	def joinWithNode ( self, node, path, makeGroup=False, cloneTransform=None ):

		'''
		Generate a SVG <path> element containing the path data "path".
		Then put this new <path> element into a <group> with the supplied
		node.  This means making a new <group> element and making the
		node a child of it with the new <path> as a sibling.
		'''

		if ( not path ) or ( len( path ) == 0 ):
			return

		if makeGroup:
			# Make a new SVG <group> element whose parent is the parent of node
			parent = node.getparent()
			#was: if not parent:
			if parent is None:
				parent = self.document.getroot()
			g = inkex.etree.SubElement( parent, inkex.addNS( 'g', 'svg' ) )

			# Move node to be a child of this new <g> element
			g.append( node )

			# Promote the node's transform to the new parent group
			# This way, it will apply to the original paths and the
			# "twisted" paths
			transform = node.get( 'transform' )
			if transform:
				g.set( 'transform', transform )
				del node.attrib['transform']
		else:
			g = node.getparent()

		# Now make a <path> element which contains the twist & is a child
		# of the new <g> element
		style = { 'stroke': '#000000', 'fill': 'none', 'stroke-width': '1' }
		line_attribs = { 'style':simplestyle.formatStyle( style ), 'd': path }
		if ( cloneTransform != None ) and ( cloneTransform != '' ):
			line_attribs['transform'] = cloneTransform
		inkex.etree.SubElement( g, inkex.addNS( 'path', 'svg' ), line_attribs )

	def twist( self, ratio ):

		if not self.paths:
			return

		# Now iterate over all of the polygons
		for path in self.paths:
			for subpath in self.paths[path]:
				for i in range( 0, len( subpath ) - 1 ):
					x = subpath[i][0] + ratio * ( subpath[i+1][0] - subpath[i][0] )
					y = subpath[i][1] + ratio * ( subpath[i+1][1] - subpath[i][1] )
					subpath[i] = [x, y]
				subpath[-1] = subpath[0]

	def draw( self, makeGroup=False ):

		'''
		Draw the edges of the current list of vertices
		'''

		if not self.paths:
			return

		# Now iterate over all of the polygons
		for path in self.paths:
			for subpath in self.paths[path]:
				pdata = ''
				for vertex in subpath:
					if pdata == '':
						pdata = 'M %f,%f' % ( vertex[0], vertex[1] )
					else:
						pdata += ' L %f,%f' %  ( vertex[0], vertex[1] )
				self.joinWithNode( path, pdata, makeGroup, self.paths_clone_transform[path] )

	def effect( self ):

		# Build a list of the vertices for the document's graphical elements
		if self.options.ids:
			# Traverse the selected objects
			for id in self.options.ids:
				self.recursivelyTraverseSvg( [self.selected[id]] )
		else:
			# Traverse the entire document
			self.recursivelyTraverseSvg( self.document.getroot() )

		# Now iterate over the vertices N times
		for n in range( 0, self.options.nSteps ):
			self.twist( self.options.fRatio )
			self.draw( n == 0 )

if __name__ == '__main__':

	e = Twist()
	e.affect()
