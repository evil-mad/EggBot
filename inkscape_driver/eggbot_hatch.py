#!/usr/bin/env python

# eggbot_hatch.py
#
# Generate hatch fills for all polygons in the current document.  The fill
# rule is an odd/even rule: odd numbered intersections (1, 3, 5, etc.)
# are a hatch line entering a polygon while even numbered intersections
# (2, 4, 6, etc.) are the same hatch line exiting the polygon.
#
# This extension first decomposes every visible <path>, <rect>, <line>,
# <polyline>, <polygon>, <circle>, and <ellipse> into individual move to
# and line to coordinates using the same procedure that eggbot.py does
# for plotting.  These coordinates are then used to build vertex lists
# for each polygon in each of the visible graphical elements (e.g., <path>,
# <rect>, etc.).  Note that a single graphical element may be composed of
# several polygons.  That is, a graphical element may contain multiple,
# disjoint paths.  We refer to each of these disjoint paths as "subpaths"
# and say that each graphical element is comprised of zero or more subpaths.
#
# The vertices for all the graphical elements that contain one or more
# subpaths are then stored in a single list named "self.vertices".  This
# single list has the format,
#
#    Graphical element 1 pointer [lxml.etree Element pointer]
#       Subpath 1, vertex 1 (x, y) coordinate [2-tuple of 2 floats]
#       Subpath 1, vertex 2 (x, y) coordinate
#       ...
#       Subpath 1, vertex N1,1 (x, y) coordinate
#    Graphical element 1 pointer
#       Subpath 2, vertex 1 (x, y) coordinate
#       Subpath 2, vertex 2 (x, y) coordinate
#       ...
#       Subpath 2, vertex N1,2 (x, y) coordinate
#    ...
#    Graphical element 2 pointer
#       Subpath 1, vertex 1 (x, y) coordinate
#       Subpath 1, vertex 2 (x, y) coordinate
#       ...
#       Subpath 1, vertex N2,1 (x, y) coordinate
#    Graphical element 2 pointer
#       Subpath 2, vertex 1 (x, y) coordinate
#       Subpath 2, vertex 2 (x, y) coordinate
#       ...
#       Subpath 2, vertex N2,2 (x, y) coordinate
#    ...
#
#  Once the list of all the vertices is built, potential hatch lines are
#  "projected" through the entire drawing.  For each potential hatch line,
#  all intersections with all the polygons are determined.  These
#  intersections are stored as decimal fractions indicating where along the
#  length of the hatch line the intersection occurs.  These values will
#  always be in the range [0, 1].  A value of 0 indicates that the
#  intersection is at the start of the hatch line, a value of 0.5 midway,
#  and a value of 1 at the end of the hatch line.
#
#  For a given hatch line, all the fractional values are sorted and any
#  duplicates removed.  Duplicates occur, for instance, when the hatch
#  line passes through a polygon vertex and thus intersects two line
#  segments of the polygon: the end of one line segment and the start of
#  another.
#
#  Once sorted and duplicates removed, an odd/even rule is applied to
#  determine which segments of the potential hatch line are within
#  polygons.  These segments found to be within polygons are then saved
#  and become the hatch fill lines which will be drawn.
#
#  With each saved hatch fill line, information about which SVG graphical
#  element it is within is saved.  This way, the hatch fill lines can
#  later be grouped with the element they are associated with.  This makes
#  it possible to manipulate the two -- graphical element and hatch lines --
#  as a single object within Inkscape.
#
#  Note: we also save the transformation matrix for each graphical element.
#  That way, when we group the hatch fills with the element they are
#  filling, we can invert the transformation.  That is, in order to compute
#  the hatch fills, we first have had apply ALL applicable transforms to
#  all the graphical elements.  We need to do that so that we know where in
#  the drawing  each of the graphical elements are relative to one another.
#  However, this means that the hatch lines have been computed in a setting
#  where no further transforms are needed.  If we then put these hatch lines
#  into the same groups as the elements being hatched in the ORIGINAL
#  drawing, then the hatch lines will get transforms applied again.  So,
#  once we compute the hatch lines, we need to invert the transforms of
#  the group they will be placed in.  Hence the need to save the transform
#  matrix for every graphical element.
#
# Written by Daniel C. Newman for the Eggbot Project
# 15 October 2010
#
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
import simpletransform
import simplestyle
import cubicsuperpath
import cspsubdiv
import bezmisc
import math

'''
Geometry 101: Determing if two lines intersect

A line L is defined by two points in space P1 and P2.  Any point P on the
line L satisfies

	P = P1 + s (P2 - P1)

for some value of the real number s in the range (-infinity, infinity).
If we confine s to the range [0, 1] then we've described the line segment
with end points P1 and P2.

Consider now the line La defined by the points P1 and P2, and the line Lb
defined by the points P3 and P4.  Any points Pa and Pb on the lines La and
Lb therefore satisfy

	Pa = P1 + sa (P2 - P1)
	Pb = P3 + sb (P4 - P3)

for some values of the real numbers sa and sb.  To see if these two lines
La and Lb intersect, we wish to see if there are finite values sa and sb
for which

	Pa = Pb

Or, equivalently,

	P1 + sa (P2 - P1) = P3 + sb (P4 - P3)

If we confine ourselves to a two-dimensional plane, and take

	P1 = (x1, y1)
	P2 = (x2, y2)
	P3 = (x3, y3)
	P4 = (x4, y4)

we then find that we have two equations in two unknowns, sa and sb,

	x1 + sa ( x2 - x1 ) = x3 + sb ( x4 - x3 )
	y1 + sa ( y2 - y1 ) = y3 + sb ( y4 - y3 )

Solving these two equations for sa and sb yields,

	sa = [ ( y1 - y3 ) ( x4 - x3 ) - ( y4 - y3 ) ( x1 - x3 ) ] / d
	sb = [ ( y1 - y3 ) ( x2 - x1 ) - ( y2 - y1 ) ( x1 - x3 ) ] / d

where the denominator, d, is given by

	d = ( y4 - y3 ) ( x2 - x1 ) - ( y2 - y1 ) ( x4 - x3 )

Substituting these back for the point (x, y) of intersection gives

	x = x1 + sa ( x2 - x1 )
	y = y1 + sa ( y2 - y1 )

Note that

1. The lines are parallel when d = 0
2. The lines are coincident d = 0 and the numerators for sa & sb are zero
3. For line segments, sa and sb are in the range [0, 1]; any value outside
   that range indicates that the line segments do not intersect.
'''

def intersect( P1, P2, P3, P4 ):

	'''
	Determine if two line segments defined by the four points P1 & P2 and
	P3 & P4 intersect.  If they do intersect, then return the fractional
	point of intersection "sa" along the first line at which the
	intersection occurs.
	'''

	# Denominator
	d = ( P4[1] - P3[1] ) * ( P2[0] - P1[0] ) - \
			( P2[1] - P1[1] ) * ( P4[0] - P3[0] )

	# Return now if the denominator is zero
	if d == 0:
		return float( -1 )

	# For our purposes, the first line segment given
	# by P1 & P2 is the LONG hatch line running through
	# the entire drawing.  And, P3 & P4 describe the
	# usually much shorter line segment from a polygon.
	# As such, we compute sb first as it's more likely
	# to indicate "no intersection".  That is, sa is
	# more likely to indicate an intersection with a
	# much a long line containing P3 & P4.

	nb = ( P1[1] - P3[1] ) * ( P2[0] - P1[0] ) - \
			( P2[1] - P1[1] ) * ( P1[0] - P3[0] )
	# Could first check if abs(nb) > abs(d) or if
	# the signs differ.
	sb = float( nb ) / float( d )
	if ( sb < 0 ) or ( sb > 1 ):
		return float( -1 )

	na = ( P1[1] - P3[1] ) * ( P4[0] - P3[0] ) - \
			( P4[1] - P3[1] ) * ( P1[0] - P3[0] )
	sa = float( na ) / float( d )
	if ( sa < 0 ) or ( sa > 1 ):
		return float( -1 )

	return sa

def interstices( P1, P2, vertices, hatches ):

	'''
	For the line L defined by the points P1 & P2, determine the segments
	of L which lie within the polygons described in the vertex list,
	"vertices".

	P1 -- (x,y) coordinate [2-tuple]
	P2 -- (x,y) coordinate [2-tuple]
	vertices -- List of vertices for each polygon.  Format as per the
				  introduction to this extension.  (See way above.)

	When an intersection of the line L is found with a polygon edge, then
	the fractional distance along the line L is saved along with the
	lxml.etree node which contained the intersecting polygon edge.  This
	fractional distance is always in the range [0, 1].

	Once all polygons have been checked, the list of fractional distances
	corresponding to intersections is sorted and any duplicates removed.
	It is then assumed that the first intersection is the line L entering
	a polygon; the second intersection the line leaving the polygon.  This
	line segment defined by the first and second intersection points is
	thus a hatch fill line we sought to generate.  In general, our hatch
	fills become the line segments described by intersection i and i+1
	with i an odd value (1, 3, 5, ...).  Since we know the lxml.etree node
	corresponding to each intersection, we can then correlate the hatch
	fill lines to the graphical elements in the original SVG document.
	This enables us to group hatch lines with the elements being hatched.

	The hatch line segments are returned by populating a dictionary.
	The dictionary is keyed off of the lxml.etree node pointer.  Each
	dictionary value is a list of 4-tuples,

		(x1, y1, x2, y2)

	where (x1, y1) and (x2, y2) are the (x,y) coordinates of the line
	segment's starting and ending points.
	'''

	# First entry in vertices is an lxml.etree node pointer
	# We don't hatch a point [a single vertex; len(vertices) == 2]
	# We don't hatch a line [only two vertices; len(vertices) == 3]
	if len( vertices ) < 3:
		return None

	sa = []

	# P1 & P2 is the hatch line
	# P3 & P4 is the polygon edge to check
	current_node = vertices[0]
	P3 = vertices[1]
	for P4 in vertices[2:]:
		if len( P4 ) == 0:
			current_node = P4
		elif len( P3 ):
			s = intersect( P1, P2, P3, P4 )
			if ( s >= 0.0 ) and ( s <= 1.0 ):
				# Save this intersection point along the hatch line
				sa.append( ( s, current_node ) )
		P3 = P4

	# Return now if there were no intersections
	if len( sa ) == 0:
		return None

	# Sort the intersections
	sa.sort()

	# Remove duplicates intersections.  A common case where these arise
	# in when the hatch line passes through a vertex where one line segment
	# ends and the next one begins.

	# Having had sorted the data, it's trivial to just scan through
	# removing duplicates as we go and then truncating the array
	n = len( sa )
	ilast = i = 1
	last = sa[0]
	while i < n:
		if abs( sa[i][0] - last[0] ) > 0.00001:
			sa[ilast] = last = sa[i]
			ilast += 1
		i += 1
	sa = sa[:ilast]
	if len( sa ) < 2:
		return

	# Now, entries with even valued indices into sa[] are where we start
	# a hatch line and odd valued indices where we end the hatch line.
	for i in range( 0, len( sa ) - 1, 2 ):
		if not hatches.has_key( sa[i][1] ):
			hatches[sa[i][1]] = []
		x1 = P1[0] + sa[i][0] * ( P2[0] - P1[0] )
		y1 = P1[1] + sa[i][0] * ( P2[1] - P1[1] )
		x2 = P1[0] + sa[i+1][0] * ( P2[0] - P1[0] )
		y2 = P1[1] + sa[i+1][0] * ( P2[1] - P1[1] )
		hatches[sa[i][1]].append( [[x1, y1], [x2, y2]] )

# Lifted with impunity from eggbot.py

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

	SVG represents the transform matrix as matrix(a b c d e f)
	while Inkscape extensions store the transform matrix as

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

def subdivideCubicPath( sp, flat, i=1 ):

	"""
	Break up a bezier curve into smaller curves, each of which
	is approximately a straight line within a given tolerance
	(the "smoothness" defined by [flat]).

	This is a modified version of cspsubdiv.cspsubdiv(). I rewrote the recursive
	call because it caused recursion-depth errors on complicated line segments.
	"""

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

class Eggbot_Hatch( inkex.Effect ):

	def __init__( self ):

		inkex.Effect.__init__( self )
		self.xmin, self.ymin = ( float( 0 ), float( 0 ) )
		self.xmax, self.ymax = ( float( 0 ), float( 0 ) )
		self.vertices = []
		self.grid = []
		self.hatches = {}
		self.transforms = {}
		self.OptionParser.add_option(
			"--crossHatch", action="store", dest="crossHatch",
			type="inkbool", default=False,
			help="Generate a cross hatch pattern" )
		self.OptionParser.add_option(
			"--hatchAngle", action="store", type="float",
			dest="hatchAngle", default=90.0,
			help="Angle of inclination for hatch lines" )
		self.OptionParser.add_option(
			"--hatchSpacing", action="store", type="float",
			dest="hatchSpacing", default=10.0,
			help="Spacing between hatch lines" )

	def addPathVertices( self, path, node=None, transform=None ):

		'''
		Decompose the path data from an SVG element into individual
		move to and line to coordinates.  Place these coordinates into a
		list of polygon vertices.  This list begins with the lxml.etree
		node pointer for the SVG element which owns the path data.  It is
		then followed by all the points (vertices) in a subpath.  When
		that subpath ends then either we're done, or there are more
		subpaths to add.  If there are more subpaths to add, then we
		again add the lxml.etree node pointer and follow it by the
		vertices from the next subpath.  This continues until we have
		exhausted all the subpaths for the SVG element.
		'''

		if ( not path ) or ( len( path ) == 0 ):
			return

		# parsePath() may raise an exception.  This is okay
		sp = simplepath.parsePath( path )
		if ( not sp ) or ( len( sp ) == 0 ):
			return

		# Get a cubic super duper path
		p = cubicsuperpath.CubicSuperPath( sp )
		if ( not p ) or ( len( p ) == 0 ):
			return

		# Apply any transformation
		if transform:
			simpletransform.applyTransformToPath( transform, p )

		# Now traverse the simplified path
		path_vertices = []
		subpath_vertices = []
		for sp in p:
			# We've started a new subpath
			# See if there is a prior subpath and whether we should keep it
			if len( subpath_vertices ):
				if distanceSquared( subpath_vertices[0], subpath_vertices[-1] ) < 1:
					# Keep the prior subpath: it appears to be a closed path
					if len( path_vertices ):
						# Put in a flag between the prior subpath and this one
						path_vertices.append( node )
					path_vertices += subpath_vertices
			subpath_vertices = []
			subdivideCubicPath( sp, float( 0.2 ) )
			for csp in sp:
				# Add this vertex to the list of vetices
				subpath_vertices.append( csp[1] )

		# Handle final subpath
		if len( subpath_vertices ):
			if distanceSquared( subpath_vertices[0], subpath_vertices[-1] ) < 1:
				# Path appears to be closed so let's keep it
				if len( path_vertices ):
					# Put in a flag between the prior subpath and this one
					path_vertices.append( node )
				path_vertices += subpath_vertices

		# Empty path?
		if len( path_vertices ) == 0:
			return

		# Start these vertices off with their lxml node pointer
		self.vertices.append( node )

		# And append this to our list of growing vertices
		self.vertices += path_vertices

		# And save the transform for this element in a dictionary keyed
		# by the element's lxml node pointer
		self.transforms[node] = transform

	def getBoundingBox( self ):

		'''
		Determine the bounding box for our collection of polygons
		'''

		if len( self.vertices ) < 2:
			return

		# First vertex is a pointer
		self.xmin, self.ymin = self.vertices[1]
		self.xmax, self.ymax = self.vertices[1]

		for v in self.vertices[2:]:

			# Skip over pointers
			if len( v ) == 0:
				continue

			if v[0] < self.xmin:
				self.xmin = v[0]
			elif v[0] > self.xmax:
				self.xmax = v[0]
			if v[1] < self.ymin:
				self.ymin = v[1]
			elif v[1] > self.ymax:
				self.ymax = v[1]

	def recursivelyTraverseSvg( self, aNodeList,
			matCurrent=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
			parent_visibility='visible' ):

		'''
		Recursively walk the SVG document, building polygon vertex lists
		for each graphical element we support.

		Rendered SVG elements:
			<circle>, <ellipse>, <line>, <path>, <polygon>, <polyline>, <rect>

		Supported SVG elements:
			<group>, <use>

		Ignored SVG elements:
			<defs>, <eggbot>, <metadata>, <namedview>, <pattern>

		All other SVG elements trigger an error (including <text>)
		'''

		for node in aNodeList:

			# Ignore invisible nodes
			v = node.get( 'visibility', parent_visibility )
			if v == 'inherit':
				v = parent_visibility
			if v == 'hidden' or v == 'collapse':
				pass

			# first apply the current matrix transform to this node's tranform
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
					tran = node.get( 'transform' )
					if tran:
						tran += ' translate(%f,%f)' % ( x, y )
					else:
						tran = 'translate(%f,%f)' % ( x, y )
					matNew2 = simpletransform.composeTransform( matNew,
						simpletransform.parseTransform( tran ) )
					v = node.get( 'visibility', v )
					self.recursivelyTraverseSvg( refnode, matNew2, parent_visibility=v )

			elif node.tag == inkex.addNS( 'path', 'svg' ):

				path_data = node.get( 'd')
				if path_data:
					self.addPathVertices( path_data, node, matNew )

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
				self.addPathVertices( simplepath.formatPath( a ), node, matNew )

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
				self.addPathVertices( simplepath.formatPath( a ), node, matNew )

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
				self.addPathVertices( d, node, matNew )

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
				self.addPathVertices( d, node, matNew )

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
					self.addPathVertices( d, node, matNew )

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

	def joinFillsWithNode ( self, node, path ):

		'''
		Generate a SVG <path> element containing the path data "path".
		Then put this new <path> element into a <group> with the supplied
		node.  This means making a new <group> element and moving node
		under it with the new <path> as a sibling element.
		'''

		if ( not path ) or ( len( path ) == 0 ):
			return

		# Make a new SVG <group> element whose parent is the parent of node
		parent = node.getparent()
		if not parent:
			parent = self.document.getroot()
		g = inkex.etree.SubElement( parent, inkex.addNS( 'g', 'svg' ) )

		# Move node to be a child of this new <g> element
		g.append( node )

		# Now make a <path> element which contains the hatches & is a child
		# of the new <g> element
		style = { 'stroke': '#000000', 'fill': 'none', 'stroke-width': '1' }
		line_attribs = { 'style':simplestyle.formatStyle( style ), 'd': path }
		inkex.etree.SubElement( g, inkex.addNS( 'path', 'svg' ), line_attribs )

	def makeHatchGrid( self, angle, spacing, init=True ):

		'''
		Build a grid of hatch lines which encompasses the entire bounding
		box of the graphical elements we are to hatch.

		1. Figure out the bounding box for all of the graphical elements
		2. Pick a rectangle larger than that bounding box so that we can
		   later rotate the rectangle and still have it cover the bounding
		   box of the graphical elements.
		3. Center the rectangle of 2 on the origin (0, 0).
		4. Build the hatch line grid in this rectangle.
		5. Rotate the rectangle by the hatch angle.
		6. Translate the center of the rotated rectangle, (0, 0), to be
		   the center of the bounding box for the graphical elements.
		7. We now have a grid of hatch lines which overlay the graphical
		   elements and can now be intersected with those graphical elements.
		'''

		# If this is the first call, do some one time initializations
		# When generating cross hatches, we may be called more than once
		if init:
			self.getBoundingBox()
			self.grid = []

		# Determine the width and height of the bounding box containing
		# all the polygons to be hatched
		w = self.xmax - self.xmin
		h = self.ymax - self.ymin

		# Now cook up a length which is more than long enough to be at
		# least as large as the radius of the circle encompassing the
		# bounding box.  We could compute the actual radius, but an
		# easy to compute larger value is just fine
		d = float( 2 * ( w + h ) )

		# Now generate hatch lines within the square
		# centered at (0, 0) and with side length at least d

		# While we could generate these lines running back and forth,
		# that makes for weird behavior later when applying odd/even
		# rules AND there are nested polygons.  Instead, when we generate
		# the SVG <path> elements with the hatch line segments, we can
		# do the back and forth weaving.

		# Rotation information
		ca = math.cos( math.radians( 90 - angle ) )
		sa = math.sin( math.radians( 90 - angle ) )

		# Translation information
		cx = self.xmin + ( w / 2 )
		cy = self.ymin + ( h / 2 )

		# Since the spacing may be fractional (e.g., 6.5), we
		# don't try to use range() or other integer iterator
		spacing = float( abs( spacing ) )
		i = -d
		while i <= d:
			# Line starts at (i,-d) and goes to (i,+d)
			x1 = cx + ( i * ca ) + ( d * sa ) #  i * ca - (-d) * sa
			y1 = cy + ( i * sa ) - ( d * ca ) #  i * sa + (-d) * ca
			x2 = cx + ( i * ca ) - ( d * sa ) #  i * ca - (+d) * sa
			y2 = cy + ( i * sa ) + ( d * ca ) #  i * sa + (+d) * ca
			i += spacing
			# We were very generous in sizing the grid.  Go ahead
			# and curtail it some
			if (( x1 < self.xmin ) and ( x2 < self.xmin )) or \
				(( x1 > self.xmax ) and ( x2 > self.xmax )):
				continue
			if (( y1 < self.ymin ) and ( y2 < self.ymin )) or \
				(( y1 > self.ymax ) and ( y2 > self.ymax )):
				continue
			self.grid.append( ( x1, y1, x2, y2 ) )

	def effect( self ):

		# Build a list of the vertices for the document's graphical elements
		self.vertices = []
		if self.options.ids:
			# Traverse the selected objects
			for id in self.options.ids:
				self.recursivelyTraverseSvg( [self.selected[id]] )
		else:
			# Traverse the entire document
			self.recursivelyTraverseSvg( self.document.getroot() )

		# Build a grid of possible hatch lines
		self.makeHatchGrid( float( self.options.hatchAngle ),
				    float( self.options.hatchSpacing ), True )
		if self.options.crossHatch:
			self.makeHatchGrid( float( self.options.hatchAngle + 90.0 ),
				float( self.options.hatchSpacing ), False )

		# Now loop over our hatch lines looking for intersections
		for h in self.grid:
			interstices( (h[0], h[1]), (h[2], h[3]), self.vertices, self.hatches )

		# Now, dump the hatch fills sorted by which document element
		# they correspond to.  This is made easy by the fact that we
		# saved the information and used each element's lxml.etree node
		# pointer as the dictionary key under which to save the hatch
		# fills for that node.

		for key in self.hatches:
			path = ''
			direction = True
			for segment in self.hatches[key]:
				pt1 = segment[0]
				pt2 = segment[1]
				# Okay, we're going to put these hatch lines into the same
				# group as the element they hatch.  That element is down
				# some chaing of SVG elements, some of which may have
				# transforms attached.  But, our hatch lines have been
				# computed assuming that those transforms have already
				# been applied (since we had to apply them so as to know
				# where this element is on the page relative to other
				# elements and their transforms).  So, we need to invert
				# the transforms for this element and then either apply
				# that inverse transform here and now or set it in a
				# transform attribute of the <path> element.  Having it
				# set in the path element seems a bit counterintuitive
				# after the fact (i.e., what's this tranform here for?).
				# So, we compute the inverse transform here AND apply it
				# here as well.
				if self.transforms.has_key( key ):
					transform = inverseTransform( self.transforms[key] )
					simpletransform.applyTransformToPoint( transform, pt1 )
					simpletransform.applyTransformToPoint( transform, pt2 )
				# Now generate the path data for the <path>
				if direction:
					# Go this direction
					path += 'M %f,%f l %f,%f ' % \
						( pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1] )
				else:
					# Or go this direction
					path += 'M %f,%f l %f,%f ' % \
						( pt2[0], pt2[1], pt1[0] - pt2[0], pt1[1] - pt2[1] )
				direction = not direction
			self.joinFillsWithNode( key, path[:-1] )

if __name__ == '__main__':

	e = Eggbot_Hatch()
	e.affect()
