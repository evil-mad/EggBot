#!/usr/bin/env python

# eggbot_hatch.py
#
# Generate hatch fills for the closed paths (polygons) in the currently
# selected document elements.  If no elements are selected, then all the
# polygons throughout the document are hatched.  The fill rule is an odd/even
# rule: odd numbered intersections (1, 3, 5, etc.) are a hatch line entering
# a polygon while even numbered intersections (2, 4, 6, etc.) are the same
# hatch line exiting the polygon.
#
# This extension first decomposes the selected <path>, <rect>, <line>,
# <polyline>, <polygon>, <circle>, and <ellipse> elements into individual
# moveto and lineto coordinates using the same procedure that eggbot.py uses
# for plotting.  These coordinates are then used to build vertex lists.
# Only the vertex lists corresponding to polygons (closed paths) are
# kept.  Note that a single graphical element may be composed of several
# subpaths, each subpath potentially a polygon.
#
# Once the lists of all the vertices are built, potential hatch lines are
# "projected" through the bounding box containing all of the vertices.
# For each potential hatch line, all intersections with all the polygon
# edges are determined.  These intersections are stored as decimal fractions
# indicating where along the length of the hatch line the intersection
# occurs.  These values will always be in the range [0, 1].  A value of 0
# indicates that the intersection is at the start of the hatch line, a value
# of 0.5 midway, and a value of 1 at the end of the hatch line.
#
# For a given hatch line, all the fractional values are sorted and any
# duplicates removed.  Duplicates occur, for instance, when the hatch
# line passes through a polygon vertex and thus intersects two edges
# segments of the polygon: the end of one edge and the start of
# another.
#
# Once sorted and duplicates removed, an odd/even rule is applied to
# determine which segments of the potential hatch line are within
# polygons.  These segments found to be within polygons are then saved
# and become the hatch fill lines which will be drawn.
#
# With each saved hatch fill line, information about which SVG graphical
# element it is within is saved.  This way, the hatch fill lines can
# later be grouped with the element they are associated with.  This makes
# it possible to manipulate the two -- graphical element and hatch lines --
# as a single object within Inkscape.
#
# Note: we also save the transformation matrix for each graphical element.
# That way, when we group the hatch fills with the element they are
# filling, we can invert the transformation.  That is, in order to compute
# the hatch fills, we first have had apply ALL applicable transforms to
# all the graphical elements.  We need to do that so that we know where in
# the drawing  each of the graphical elements are relative to one another.
# However, this means that the hatch lines have been computed in a setting
# where no further transforms are needed.  If we then put these hatch lines
# into the same groups as the elements being hatched in the ORIGINAL
# drawing, then the hatch lines will have transforms applied again.  So,
# once we compute the hatch lines, we need to invert the transforms of
# the group they will be placed in and apply this inverse transform to the
# hatch lines.  Hence the need to save the transform matrix for every
# graphical element.

# Written by Daniel C. Newman for the Eggbot Project
# dan dot newman at mtbaldy dot us
# Last updated 28 November 2010
# 15 October 2010

# Updated by Windell H. Oskay, 6/14/2012
# Added tolerance parameter

# Update by Daniel C. Newman, 6/20/2012
# Add min span/gap width

# Updated by Windell H. Oskay, 1/8/2016
# Added live preview and correct issue with nonzero min gap 
# https://github.com/evil-mad/EggBot/issues/32

# Updated by Sheldon B. Michaels, 1/11/2016 thru 3/15/2016
# shel at shel dot net
# Added feature: Option to inset the hatch segments from boundaries
# Added feature: Option to join hatch segments that are "nearby", to minimize pen lifts
# The joins are made using cubic Bezier segments.
# https://github.com/evil-mad/EggBot/issues/36
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
import plot_utils		# https://github.com/evil-mad/plotink


N_PAGE_WIDTH = 3200
N_PAGE_HEIGHT = 800

F_MINGAP_SMALL_VALUE = 0.0000000001		# Was 0.00001 in the original version which did not have joined lines.
										# Reducing this by a factor of 10^5 decreased probability of occurrence of
										# the bug in the original, which got confused when the path barely
										# grazed a corner.
BEZIER_OVERSHOOT_MULTIPLIER = 0.75		# evaluation of cubic Bezier curve equation value,
										# at x = 0, with
										# endpoints at ( -0.5, 0 ), ( +0.5, 0 )
										# and control points at ( -0.5, 1.0 ), ( +0.5, 1.0 )

RADIAN_TOLERANCE_FOR_COLINEAR = 0.1		# Pragmatically adjusted to allow adjacent segments from the same scan line, even short ones,
										# to be classified as having the same angle
RADIAN_TOLERANCE_FOR_ALTERNATING_DIRECTION = 0.1	# Pragmatic adjustment again, as with colinearity tolerance
RECURSION_LIMIT = 500					# Pragmatic - if too high, risk runtime python error; if too low, miss some chances for reducing pen lifts
EXTREME_POSITIVE_NUMBER = float( 1.0E70 )
EXTREME_NEGATIVE_NUMBER = float( -1.0E70 )
MIN_HATCH_LENGTH_AS_FRACTION_OF_HATCH_SPACING = 0.25		# set minimum hatch length to some function
															# (e.g. this multiplier) of the hatch spacing
'''
Geometry 101: Determining if two lines intersect

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

Or, equivalently, we ask if there exists values of sa and sb for which
the equation

	P1 + sa (P2 - P1) = P3 + sb (P4 - P3)

holds.  If we confine ourselves to a two-dimensional plane, and take

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

	# Precompute these values -- note that we're basically shifting from
	#
	#		P = P1 + s (P2 - P1)
	#
	# to
	#
	# 		P = P1 + s D
	#
	# where D is a direction vector.  The solution remains the same of
	# course.  We'll just be computing D once for each line rather than
	# computing it a couple of times.

	D21x = P2[0] - P1[0]
	D21y = P2[1] - P1[1]
	D43x = P4[0] - P3[0]
	D43y = P4[1] - P3[1]

	# Denominator
	d = D21x * D43y - D21y * D43x

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

	nb = ( P1[1] - P3[1] ) * D21x - ( P1[0] - P3[0] ) * D21y

	# Could first check if abs(nb) > abs(d) or if
	# the signs differ.
	sb = float( nb ) / float( d )
	if ( sb < 0 ) or ( sb > 1 ):
		return float( -1 )

	na = ( P1[1] - P3[1] ) * D43x -  ( P1[0] - P3[0] ) * D43y
	sa = float( na ) / float( d )
	if ( sa < 0 ) or ( sa > 1 ):
		return float( -1 )

	return sa

def interstices( self, P1, P2, paths, hatches, bHoldBackHatches, fHoldBackSteps ):

	'''
	For the line L defined by the points P1 & P2, determine the segments
	of L which lie within the polygons described by the paths stored in
	"paths"

	P1 -- (x,y) coordinate [list]
	P2 -- (x,y) coordinate [list]
	paths -- Dictionary of all the paths to check for intersections

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

	dAndA = []
	# P1 & P2 is the hatch line
	# P3 & P4 is the polygon edge to check
	for path in paths:
		for subpath in paths[path]:
			P3 = subpath[0]
			for P4 in subpath[1:]:
				s = intersect( P1, P2, P3, P4 )
				if ( s >= 0.0 ) and ( s <= 1.0 ):
					# Save this intersection point along the hatch line
					if bHoldBackHatches:
						# We will need to know how the hatch meets the polygon segment, so that we can
						# calculate the end of a shorter line that stops short
						# of the polygon segment.
						# We compute the angle now while we have the information required,
						# but do _not_ apply it now, as we need the real,original, intersects
						# for the odd/even inside/outside operations yet to come.
						# Note that though the intersect() routine _could_ compute the join angle,
						# we do it here because we go thru here much less often than we go thru intersect().
						angleHatchRadians = math.atan2( -( P2[1] - P1[1] ) ,( P2[0] - P1[0] ) )			# from P1 toward P2, cartesian coordinates
						angleSegmentRadians = math.atan2( -( P4[1] - P3[1] ) ,( P4[0] - P3[0] ) )		# from P3 toward P4, cartesian coordinates
						angleDifferenceRadians = angleHatchRadians - angleSegmentRadians
						# coerce to range -pi to +pi
						if ( angleDifferenceRadians > math.pi ):
							angleDifferenceRadians -= 2 * math.pi
						elif (angleDifferenceRadians < -math.pi ):
							angleDifferenceRadians += 2 * math.pi
						fSinOfJoinAngle = math.sin( angleDifferenceRadians )
						fAbsSinOfJoinAngle = abs( fSinOfJoinAngle )
						if (fAbsSinOfJoinAngle != 0.0):								# Worrying about case of intersecting a segment parallel to the hatch
							fPreliminaryLengthToBeRemovedFromPt = fHoldBackSteps / fAbsSinOfJoinAngle
							bUnconditionallyExciseHatch = False
						else:	# if (fSinOfJoinAngle != 0.0):
							bUnconditionallyExciseHatch = True
						# if (fAbsSinOfJoinAngle != 0.0): else:
						
						if ( not bUnconditionallyExciseHatch):
							# if ( fPreliminaryLengthToBeRemovedFromPt > ( distance from intersection to relevant end + fHoldbackSteps ) ):
							#	  fFinalLengthToBeRemovedFromPt = ( distance from intersection to relevant end + fHoldbackSteps )
							# The relevant end of the segment is the end from which the hatch approaches at an acute angle.
							I = [0,0]
							I[0] = P1[0] + s * ( P2[0] - P1[0] )		# compute intersection point of hatch with segment
							I[1] = P1[1] + s * ( P2[1] - P1[1] )		# intersecting hatch line starts at P1, vectored toward P2,
																		#	but terminates at intersection I
							# Note that atan2 returns answer in range -pi to pi
							# Which end is the approach end of the hatch to the segment?
							# The dot product tells the answer:
							#	if dot product is positive, P2 is at the P4 end,
							#	else P2 is at the P3 end
							# We really don't need to take the time to actually take
							# 	the cosine of the angle, we are just interested in
							#	the quadrant within which the angle lies.
							# I'm sure there is an elegant way to do this, but I'll settle for results just now.
							# If the angle is in quadrants I or IV then P4 is the relevant end, otherwise P3 is
							# nb: Y increases down, rather than up
							# nb: difference angle has been forced to the range -pi to +pi
							if ( abs(angleDifferenceRadians) < math.pi / 2 ):
								# It's near the P3 the relevant end from which the hatch departs
								fDistanceFromIntersectionToRelevantEnd = math.hypot( P3[0] - I[0], P3[1] - I[1] )
								fDistanceFromIntersectionToIrrelevantEnd = math.hypot( P4[0] - I[0], P4[1] - I[1] )
							else:	# if ( abs(angleDifferenceRadians) < math.pi / 2 )
								# It's near the P4 end from which the hatch departs
								fDistanceFromIntersectionToRelevantEnd = math.hypot( P4[0] - I[0], P4[1] - I[1] )
								fDistanceFromIntersectionToIrrelevantEnd = math.hypot( P3[0] - I[0], P3[1] - I[1] )
							# if ( abs(angleDifferenceRadians) < math.pi / 2 ): else:

							# Now, the problem defined in issue 22 is that we may not need to remove the
							# entire preliminary length we've calculated.  This problem occurs because
							# we have so far been considering the polygon segment as a line of infinite extent.
							# Thus, we may be holding back at a point where no holdback is required, when
							# calculated holdback is well beyond the position of the segment end.
							
							# To make matters worse, we do not currently know whether we're
							# starting a hatch or terminating a hatch, because the duplicates have
							# yet to be removed.  All we can do then, is calculate the required
							# line shortening for both possibilities - and then choose the correct
							# one after duplicate-removal, when actually finalizing the hatches.
							
							# Let's see if either end, or perhaps both ends, has a case of excessive holdback
							
							# First, default assumption is that neither end has excessive holdback
							fFinalLengthToBeRemovedFromPtWhenStartingHatch = fPreliminaryLengthToBeRemovedFromPt
							fFinalLengthToBeRemovedFromPtWhenEndingHatch = fPreliminaryLengthToBeRemovedFromPt
							
							# Now check each of the two ends
							if ( fPreliminaryLengthToBeRemovedFromPt > ( fDistanceFromIntersectionToRelevantEnd + fHoldBackSteps ) ):
								# Yes, would be excessive holdback approaching from this direction
								fFinalLengthToBeRemovedFromPtWhenStartingHatch = fDistanceFromIntersectionToRelevantEnd + fHoldBackSteps
							# if ( fPreliminaryLengthToBeRemovedFromPt > ( fDistanceFromIntersectionToRelevantEnd + fHoldBackSteps ) ):
							if ( fPreliminaryLengthToBeRemovedFromPt > ( fDistanceFromIntersectionToIrrelevantEnd + fHoldBackSteps ) ):
								# Yes, would be excessive holdback approaching from other direction
								fFinalLengthToBeRemovedFromPtWhenEndingHatch = fDistanceFromIntersectionToIrrelevantEnd + fHoldBackSteps
							# if ( fPreliminaryLengthToBeRemovedFromPt > ( fDistanceFromIntersectionToIrrelevantEnd + fHoldBackSteps ) ):

							dAndA.append( ( s, path, fFinalLengthToBeRemovedFromPtWhenStartingHatch, fFinalLengthToBeRemovedFromPtWhenEndingHatch ) )
						else:	# if ( not bUnconditionallyExciseHatch):
							dAndA.append( ( s, path, 123456.0, 123456.0 ) )			# Mark for complete hatch excision, hatch is parallel to segment
																			# Just a random number guaranteed large enough to be longer than any hatch length
						# if ( not bUnconditionallyExciseHatch): else :
					else:	# if bHoldBackHatches:
						dAndA.append( ( s, path, 0, 0 ) )			# zero length to be removed from hatch
					# if bHoldBackHatches: else:
				# if ( s >= 0.0 ) and ( s <= 1.0 ):	
				P3 = P4
			# for P4 in subpath[1:]:
		# for subpath in paths[path]:
	# for path in paths:

	# Return now if there were no intersections
	if len( dAndA ) == 0:
		return None

	dAndA.sort()
	
	# Remove duplicate intersections.  A common case where these arise
	# is when the hatch line passes through a vertex where one line segment
	# ends and the next one begins.

	# Having sorted the data, it's trivial to just scan through
	# removing duplicates as we go and then truncating the array
	
	n = len( dAndA )
	ilast = i = 1
	last = dAndA[0]
	while i < n:
		if ( ( abs( dAndA[i][0] - last[0] ) ) > F_MINGAP_SMALL_VALUE ):
			dAndA[ilast] = last = dAndA[i]
			ilast += 1
		i += 1
	dAndA = dAndA[:ilast]
	if len( dAndA ) < 2:
		return

	# Now, entries with even valued indices into sa[] are where we start
	# a hatch line and odd valued indices where we end the hatch line.

	last_dAndA = None
	i = 0
	while i < ( len( dAndA ) - 1 ):
	#for i in range( 0, len( sa ) - 1, 2 ):
		if not hatches.has_key( dAndA[i][1] ):
			hatches[dAndA[i][1]] = []

		x1 = P1[0] + dAndA[i][0] * ( P2[0] - P1[0] )
		y1 = P1[1] + dAndA[i][0] * ( P2[1] - P1[1] )
		x2 = P1[0] + dAndA[i+1][0] * ( P2[0] - P1[0] )
		y2 = P1[1] + dAndA[i+1][0] * ( P2[1] - P1[1] )
		
		# These are the hatch ends if we are _not_ holding off from the boundary.
		if not bHoldBackHatches:
			hatches[dAndA[i][1]].append( [[x1, y1], [x2, y2]] )
		else:	# if not bHoldBackHatches:
			# User wants us to perform a pseudo inset operation.
			# We will accomplish this by trimming back the ends of the hatches.
			# The amount by which to trim back depends on the angle between the
			# intersecting hatch line with the intersecting polygon segment, and
			# may well be different at the two different ends of the hatch line.
			
			# To visualize this, imagine a hatch intersecting a segment that is
			# close to parallel with it.  The length of the hatch would have to be
			# drastically reduced in order that its closest approach to the
			# segment be reduced to the desired distance.
			
			# Imagine a Cartesian coordinate system, with the X axis representing the
			# polygon segment, and a line running through the origin with a small
			# positive slope being the intersecting hatch line.
			
			# We see that we want a Y value of the specified hatch width, and that
			# at that Y, the distance from the origin to that point is the
			# hypotenuse of the triangle.
			# 	Y / cutlength = sin(angle)
			# therefore:
			#	cutlength = Y / sin(angle)
			# Fortunately, we have already stored this angle for exactly this purpose.
			# For each end, trim back the hatch line by the amount required by
			# its own angle.  If the resultant diminished hatch is too short,
			# remove it from consideration by marking it as already drawn - a
			# fiction, but is much quicker than actually removing the hatch from the list.
			
			fMinAllowedHatchLength = self.options.hatchSpacing * MIN_HATCH_LENGTH_AS_FRACTION_OF_HATCH_SPACING
			fInitialHatchLength = math.hypot( x2 - x1, y2 - y1 )
			# We did as much as possible of the inset operation back when we were finding intersections.
			# We did it back then because at that point we knew more about the geometry than we know now.
			# Now we don't know where the ends of the segments are, so we can't address issue 22 here.
			fLengthToBeRemovedFromPt1 = dAndA[i][3]
			fLengthToBeRemovedFromPt2 = dAndA[i+1][2]
			
			if ( ( fInitialHatchLength - ( fLengthToBeRemovedFromPt1 + fLengthToBeRemovedFromPt2 ) )		\
					<=																						\
					fMinAllowedHatchLength ):
				pass								# Just don't insert it into the hatch list
			else:	# if (...too short...):
				'''
				Use:
				def RelativeControlPointPosition( self, distance, fDeltaX, fDeltaY, deltaX, deltaY ):
					# returns the point, relative to 0, 0 offset by deltaX, deltaY,
					# which extends a distance of "distance" at a slope defined by fDeltaX and fDeltaY
				'''
				pt1 = self.RelativeControlPointPosition( fLengthToBeRemovedFromPt1, x2 - x1, y2 - y1, x1, y1 )
				pt2 = self.RelativeControlPointPosition( fLengthToBeRemovedFromPt2, x1 - x2, y1 - y2, x2, y2 )
				hatches[dAndA[i][1]].append( [[pt1[0], pt1[1]], [pt2[0], pt2[1]]] )

			# if (...too short...): else:
		# if not bHoldBackHatches: else:
		
		# Remember the relative start and end of this hatch segment
		last_dAndA = [ dAndA[i], dAndA[i+1] ]

		i = i + 2
	# while i < ( len( dAndA ) - 1 ):
	
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

def subdivideCubicPath( sp, flat, i=1 ):

	"""
	Break up a bezier curve into smaller curves, each of which
	is approximately a straight line within a given tolerance
	(the "smoothness" defined by [flat]).

	This is a modified version of cspsubdiv.cspsubdiv() rewritten
	to avoid recurrence.
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
		self.paths = {}
		self.grid = []
		self.hatches = {}
		self.transforms = {}

		# For handling an SVG viewbox attribute, we will need to know the
		# values of the document's <svg> width and height attributes as well
		# as establishing a transform from the viewbox to the display.
		self.docWidth = float( N_PAGE_WIDTH )
		self.docHeight = float( N_PAGE_HEIGHT )
		self.docTransform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]


		self.OptionParser.add_option(
			"--holdBackSteps", action="store", type="float",
			dest="holdBackSteps", default=3.0,
			help="How far hatch strokes stay from boundary (steps)" )
		self.OptionParser.add_option(
			"--hatchScope", action="store", type="float",
			dest="hatchScope", default=3.0,
			help="Radius searched for segments to join (units of hatch width)" )
		self.OptionParser.add_option(
			"--holdBackHatchFromEdges", action="store", dest="holdBackHatchFromEdges",
			type="inkbool", default=True,
			help="Stay away from edges, so no need for inset" )
		self.OptionParser.add_option(
			"--reducePenLifts", action="store", dest="reducePenLifts",
			type="inkbool", default=True,
			help="Reduce plotting time by joining some hatches" )
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
		self.OptionParser.add_option(
			"--tolerance", action="store", type="float",
			dest="tolerance", default=20.0,
			help="Allowed deviation from original paths" )
		self.OptionParser.add_option( "--tab",	#NOTE: value is not used.
			action="store", type="string", dest="tab", default="splash",
			help="The active tab when Apply was pressed" )

	def getDocProps( self ):

		'''
		Get the document's height and width attributes from the <svg> tag.
		Use a default value in case the property is not present or is
		expressed in units of percentages.
		'''

		self.docHeight = plot_utils.getLength( self, 'height', N_PAGE_HEIGHT )
		self.docWidth = plot_utils.getLength( self, 'width', N_PAGE_WIDTH )
		
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

	def addPathVertices( self, path, node=None, transform=None ):

		'''
		Decompose the path data from an SVG element into individual
		subpaths, each starting with an absolute move-to (x, y)
		coordinate followed by one or more absolute line-to (x, y)
		coordinates.  Each subpath is stored as a list of (x, y)
		coordinates, with the first entry understood to be a
		move-to coordinate and the rest line-to coordinates.  A list
		is then made of all the subpath lists and then stored in the
		self.paths dictionary using the path's lxml.etree node pointer
		as the dictionary key.
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
		if transform != None:
			simpletransform.applyTransformToPath( transform, p )

		# Now traverse the simplified path
		subpaths = []
		subpath_vertices = []
		for sp in p:
			# We've started a new subpath
			# See if there is a prior subpath and whether we should keep it
			if len( subpath_vertices ):
				if distanceSquared( subpath_vertices[0], subpath_vertices[-1] ) < 1:
					# Keep the prior subpath: it appears to be a closed path
					subpaths.append( subpath_vertices )
			subpath_vertices = []
			subdivideCubicPath( sp, float( self.options.tolerance / 100 ) )
			for csp in sp:
				# Add this vertex to the list of vertices
				subpath_vertices.append( csp[1] )

		# Handle final subpath
		if len( subpath_vertices ):
			if distanceSquared( subpath_vertices[0], subpath_vertices[-1] ) < 1:
				# Path appears to be closed so let's keep it
				subpaths.append( subpath_vertices )

		# Empty path?
		if len( subpaths ) == 0:
			return

		# And add this path to our dictionary of paths
		self.paths[node] = subpaths

		# And save the transform for this element in a dictionary keyed
		# by the element's lxml node pointer
		self.transforms[node] = transform

	def getBoundingBox( self ):

		'''
		Determine the bounding box for our collection of polygons
		'''

		self.xmin, self.xmax = EXTREME_POSITIVE_NUMBER, EXTREME_NEGATIVE_NUMBER
		self.ymin, self.ymax = EXTREME_POSITIVE_NUMBER, EXTREME_NEGATIVE_NUMBER
		for path in self.paths:
			for subpath in self.paths[path]:
				for vertex in subpath:
					if vertex[0] < self.xmin:
						self.xmin = vertex[0]
					elif vertex[0] > self.xmax:
						self.xmax = vertex[0]
					if vertex[1] < self.ymin:
						self.ymin = vertex[1]
					elif vertex[1] > self.ymax:
						self.ymax = vertex[1]
				# for vertex in subpath:
			# for subpath in self.paths[path]:
		# for path in self.paths:

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
					# Note: the transform has already been applied
					if ( x != 0 ) or ( y != 0 ):
						matNew2 = composeTransform( matNew, parseTransform( 'translate(%f,%f)' % (x,y) ) )
					else:
						matNew2 = matNew
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
		# for node in aNodeList:
	# def recursivelyTraverseSvg( self, aNodeList,...
	
	def joinFillsWithNode ( self, node, stroke_width, path ):

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
		#was: if not parent:
		if parent is None:
			parent = self.document.getroot()
		g = inkex.etree.SubElement( parent, inkex.addNS( 'g', 'svg' ) )
		# Move node to be a child of this new <g> element
		g.append( node )

		# Now make a <path> element which contains the hatches & is a child
		# of the new <g> element
		stroke_color = '#000000'		# default assumption
		stroke_width = '1.0'		# default value
		
		try:
			style = node.get('style')
			if style != None:
				declarations = style.split(';')
				for i,declaration in enumerate(declarations):
					parts = declaration.split(':', 2)
					if len(parts) == 2:
						(prop, val) = parts
						prop = prop.strip().lower()
						if prop == 'stroke-width':
							stroke_width = val.strip()
						elif prop == 'stroke':
							val = val.strip()
							stroke_color = val
				# for i,declaration in enumerate(declarations):
			# if style != 'none':
		finally:
			style = { 'stroke': '%s' % stroke_color, 'fill': 'none', 'stroke-width': '%s' % stroke_width }
			line_attribs = { 'style':simplestyle.formatStyle( style ), 'd': path }
			tran = node.get( 'transform' )
			if ( tran != None ) and ( tran != '' ):
				line_attribs['transform'] = tran
			inkex.etree.SubElement( g, inkex.addNS( 'path', 'svg' ), line_attribs )

	def makeHatchGrid( self, angle, spacing, init=True ):	# returns True if succeeds in making grid, else False

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
		
		bBoundingBoxExists = ( ( w != ( EXTREME_NEGATIVE_NUMBER - EXTREME_POSITIVE_NUMBER ) ) and ( h != ( EXTREME_NEGATIVE_NUMBER - EXTREME_POSITIVE_NUMBER ) ) )
		retValue = bBoundingBoxExists

		if bBoundingBoxExists:
			# Nice thing about rectangles is that the diameter of the circle
			# encompassing them is the length the rectangle's diagonal...
			r = math.sqrt ( w * w + h * h ) / 2.0

			# Length of a hatch line will be 2r
			# Now generate hatch lines within the square
			# centered at (0, 0) and with side length at least d

			# While we could generate these lines running back and forth,
			# that makes for weird behavior later when applying odd/even
			# rules AND there are nested polygons.  Instead, when we
			# generate the SVG <path> elements with the hatch line
			# segments, we can do the back and forth weaving.

			# Rotation information
			ca = math.cos( math.radians( 90 - angle ) )
			sa = math.sin( math.radians( 90 - angle ) )

			# Translation information
			cx = self.xmin + ( w / 2 )
			cy = self.ymin + ( h / 2 )

			# Since the spacing may be fractional (e.g., 6.5), we
			# don't try to use range() or other integer iterator
			spacing = float( abs( spacing ) )
			i = -r
			while i <= r:
				# Line starts at (i, -r) and goes to (i, +r)
				x1 = cx + ( i * ca ) + ( r * sa ) #  i * ca - (-r) * sa
				y1 = cy + ( i * sa ) - ( r * ca ) #  i * sa + (-r) * ca
				x2 = cx + ( i * ca ) - ( r * sa ) #  i * ca - (+r) * sa
				y2 = cy + ( i * sa ) + ( r * ca ) #  i * sa + (+r) * ca
				i += spacing
				# Remove any potential hatch lines which are entirely
				# outside of the bounding box
				if (( x1 < self.xmin ) and ( x2 < self.xmin )) or \
					(( x1 > self.xmax ) and ( x2 > self.xmax )):
					continue
				if (( y1 < self.ymin ) and ( y2 < self.ymin )) or \
					(( y1 > self.ymax ) and ( y2 > self.ymax )):
					continue
				self.grid.append( ( x1, y1, x2, y2 ) )
			# while i <= r:
		# if bBoundingBoxExists:
		return retValue
	# def makeHatchGrid( self, angle, spacing, init=True ):
	
	def effect( self ):
	
		global referenceCount
		global ptLastPositionAbsolute
		# Viewbox handling
		self.handleViewBox()
		
		referenceCount = 0
		ptLastPositionAbsolute = [0,0]

		# Build a list of the vertices for the document's graphical elements
		if self.options.ids:
			# Traverse the selected objects
			for id in self.options.ids:
				self.recursivelyTraverseSvg( [self.selected[id]], self.docTransform )
		else:
			# Traverse the entire document
			self.recursivelyTraverseSvg( self.document.getroot(), self.docTransform )

		# Build a grid of possible hatch lines
		bHaveGrid = self.makeHatchGrid( float( self.options.hatchAngle ),
				float( self.options.hatchSpacing ), True )
			# makeHatchGrid returns false if could not make grid - probably because bounding box is non-existent
		if bHaveGrid:
			if self.options.crossHatch:
				self.makeHatchGrid( float( self.options.hatchAngle + 90.0 ),
					float( self.options.hatchSpacing ), False )
			# if self.options.crossHatch:
			
			# Now loop over our hatch lines looking for intersections
			for h in self.grid:
				interstices( self, (h[0], h[1]), (h[2], h[3]), self.paths, self.hatches, self.options.holdBackHatchFromEdges, self.options.holdBackSteps )
				
			# Target stroke width will be (doc width + doc height) / 2 / 1000
			# stroke_width_target = ( self.docHeight + self.docWidth ) / 2000
			# stroke_width_target = 1
			stroke_width_target = 1
			# Each hatch line stroke will be within an SVG object which may
			# be subject to transforms.  So, on an object by object basis,
			# we need to transform our target width to a width suitable
			# for that object (so that after the object and its hatches are
			# transformed, the result has the desired width).

			# To aid in the process, we use a diagonal line segment of length
			# stroke_width_target.  We then run this segment through an object's
			# inverse transform and see what the resulting length of the inversely
			# transformed segment is.  We could, alternatively, look at the
			# x and y scaling factors in the transform and average them.
			s = stroke_width_target / math.sqrt( 2 )

			# Now, dump the hatch fills sorted by which document element
			# they correspond to.  This is made easy by the fact that we
			# saved the information and used each element's lxml.etree node
			# pointer as the dictionary key under which to save the hatch
			# fills for that node.
			
			absoluteLineSegments = {}
			nAbsoluteLineSegmentTotal = 0
			nPenLifts = 0
			# To implement
			for key in self.hatches:
				direction = True
				if self.transforms.has_key( key ):
					transform = inverseTransform( self.transforms[key] )
					# Determine the scaled stroke width for a hatch line
					# We produce a line segment of unit length, transform
					# its endpoints and then determine the length of the
					# resulting line segment.
					pt1 = [0, 0]
					pt2 = [s, s]
					simpletransform.applyTransformToPoint( transform, pt1 )
					simpletransform.applyTransformToPoint( transform, pt2 )
					dx = pt2[0] - pt1[0]
					dy = pt2[1] - pt1[1]
					stroke_width = math.sqrt( dx * dx + dy * dy )
				else:
					transform = None
					stroke_width = float( 1.0 )

				# The transform also applies to the hatch spacing we use when searching for end connections
				transformedHatchSpacing = stroke_width * self.options.hatchSpacing
				
				path = ''								# regardless of whether or not we're reducing pen lifts
				ptLastPositionAbsolute = [ 0,0 ]
				ptLastPositionAbsolute[0] = 0
				ptLastPositionAbsolute[1] = 0
				fDistanceMovedWithPenUp = 0
				if not self.options.reducePenLifts:
					for segment in self.hatches[key]:
						if len( segment ) < 2:
							continue
						pt1 = segment[0]
						pt2 = segment[1]
						# Okay, we're going to put these hatch lines into the same
						# group as the element they hatch.  That element is down
						# some chain of SVG elements, some of which may have
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
						# So, we compute the inverse transform and apply it here.
						if transform != None:
							simpletransform.applyTransformToPoint( transform, pt1 )
							simpletransform.applyTransformToPoint( transform, pt2 )
						# Now generate the path data for the <path>
						if direction:
							# Go this direction
							path += ( 'M %f,%f l %f,%f ' %
								( pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1] ) )
						else:
							# Or go this direction
							path += ( 'M %f,%f l %f,%f ' %
								( pt2[0], pt2[1], pt1[0] - pt2[0], pt1[1] - pt2[1] ) )
								
						direction = not direction
					# for segment in self.hatches[key]:
					self.joinFillsWithNode( key, stroke_width, path[:-1] )

				else:		# if not self.options.reducePenLifts:
					for segment in self.hatches[key]:
						if len( segment ) < 2:			# Copied from original, no idea why this is needed [sbm]
							continue
						if ( direction ):
							pt1 = segment[0]
							pt2 = segment[1]
						else:
							pt1 = segment[1]
							pt2 = segment[0]
						# Okay, we're going to put these hatch lines into the same
						# group as the element they hatch.  That element is down
						# some chain of SVG elements, some of which may have
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
						# So, we compute the inverse transform and apply it here.
						if transform != None:
							simpletransform.applyTransformToPoint( transform, pt1 )
							simpletransform.applyTransformToPoint( transform, pt2 )

						# Now generate the path data for the <path> 
						# BUT we want to combine as many paths as possible to reduce pen lifts.
						# In order to combine paths, we need to know all of the path segments.
						# The solution to this conundrum is to generate all path segments,
						# but instead of drawing them into the path right away, we put them in
						# an array where they'll be available for random access
						# by our anti-pen-lift algorithm
						absoluteLineSegments[ nAbsoluteLineSegmentTotal ] = [ pt1, pt2, False ]		# False indicates that segment has not yet been drawn
						nAbsoluteLineSegmentTotal += 1
						direction = not direction
					# for segment in self.hatches[key]:
					
					# Now have a nice juicy buffer full of line segments with absolute coordinates
					fProposedNeighborhoodRadiusSquared = self.ProposeNeighborhoodRadiusSquared( transformedHatchSpacing )	# Just fixed and simple for now - may make function of neighborhood later
					for referenceCount in range( nAbsoluteLineSegmentTotal ):	# This is the entire range of segments,
						# Sets global referenceCount to segment which has an end closest to current pen position.
						# Doesn't need to select which end is closest, as that will happen below, with nReferenceEndIndex.
						# When we have gone thru this whole range, we will be completely done.
						# We only get here again, after all _connected_ segments have been "drawn".
						if ( not absoluteLineSegments[referenceCount][2] ):		# Test whether this segment has been drawn
							# Has not been drawn yet
							
							# Before we do any irrevocable changes to path, let's see if we are going to be able to append any segments.
							# The below solution is inelegant, but has the virtue of being relatively simple to implement.
							# Pre-qualify this segment on the issue of whether it has any connecting segments.
							# If it does not, then just add the path for this one segment, and go on to the next.
							# If it does have connecting segments, we need to go through the recursive logic.
							# Lazily, again, select the desired direction of line ahead of time.
							
							bFoundSegmentToAdd = False							# default assumption
							nReferenceEndIndexAtClosest = 0
							nInnerCountAtClosest = -1
							fClosestDistanceSquared = 123456				# just a random large number
							for nReferenceEndIndex in range( 2 ):
								ptReference = absoluteLineSegments[referenceCount][nReferenceEndIndex]
								ptReferenceOtherEnd = absoluteLineSegments[referenceCount][not nReferenceEndIndex]
								fReferenceDirectionRadians = math.atan2( ptReferenceOtherEnd[1] - ptReference[1], ptReferenceOtherEnd[0] - ptReference[0] )		# from other end to this end
								# The following is just a simple copy from the routine in recursivelyAppendNearbySegmentIfAny procedure
								# Look through all possibilities to choose the closest that fulfills all requirements e.g. direction and colinearity
								for innerCount in range( nAbsoluteLineSegmentTotal ):	# investigate all segments
									if ( not absoluteLineSegments[innerCount][2] ):
										# This segment currently undrawn, so it is a candidate for a path extension
										# Need to check both ends of each and every proposed segment so we can find the most appropriate one
										# Define pt2 in the reference as the end which we want to extend
										for nNewSegmentInitialEndIndex in range( 2 ):
											# First try initial end of test segment (aka pt1) vs final end (aka pt2) of reference segment
											if ( innerCount != referenceCount ):	# don't investigate self ends
												deltaX = absoluteLineSegments[innerCount][nNewSegmentInitialEndIndex][0] - ptReference[0]		# proposed initial pt1 X minus existing final pt1 X
												deltaY = absoluteLineSegments[innerCount][nNewSegmentInitialEndIndex][1] - ptReference[1]		# proposed initial pt1 Y minus existing final pt1 Y
												if 	( ( deltaX * deltaX + deltaY * deltaY ) < fProposedNeighborhoodRadiusSquared ):
													fThisDistanceSquared = deltaX * deltaX + deltaY * deltaY
													ptNewSegmentThisEnd = absoluteLineSegments[innerCount][nNewSegmentInitialEndIndex]
													ptNewSegmentOtherEnd = absoluteLineSegments[innerCount][not nNewSegmentInitialEndIndex]
													fNewSegmentDirectionRadians = math.atan2( ptNewSegmentThisEnd[1] - ptNewSegmentOtherEnd[1], ptNewSegmentThisEnd[0] - ptNewSegmentOtherEnd[0] )		# from other end to this end
													# If this end would cause an alternating direction,
													# then exclude it
													if ( not self.WouldBeAnAlternatingDirection( fReferenceDirectionRadians, fNewSegmentDirectionRadians ) ):
														pass
	#													break		# out of for nNewSegmentInitialEndIndex in range( 2 ):
													# if ( not self.WouldBeAnAlternatingDirection( fReferenceDirectionRadians, fNewSegmentDirectionRadians ) ):
													elif ( fThisDistanceSquared < fClosestDistanceSquared ):
														# One other thing could rule out choosing this segment end:
														# Want to screen and remove two segments that, while close enough,
														# should be disqualified because they are colinear.  The reason for this is that
														# if they are colinear, they arose from the same global grid line, which means
														# that the gap between them arises from intersections with the boundary.
														# The idea here is that, all things being more-or-less equal,
														# we would like to give preference to connecting to a segment
														# which is the reverse of our current direction.  This makes for better
														# bezier curve join.
														# The criterion for being colinear is that the reference segment angle is effectively
														# the same as the line connecting the reference segment to the end of the new segment.
														fJoinerDirectionRadians = math.atan2( ptNewSegmentThisEnd[1] - ptReference[1], ptNewSegmentThisEnd[0] - ptReference[0] )
														if ( not self.AreCoLinear( fReferenceDirectionRadians, fJoinerDirectionRadians) ):
															# not colinear
															fClosestDistanceSquared = fThisDistanceSquared
															bFoundSegmentToAdd = True
															nReferenceEndIndexAtClosest = nReferenceEndIndex
															nInnerCountAtClosest = innerCount
															deltaXAtClosest = deltaX
															deltaYAtClosest = deltaY
														# if ( not self.AreCoLinear( fReferenceDirectionRadians, fJoinerDirectionRadians) ):
													# if ( fThisDistanceSquared < fClosestDistanceSquared ):
												# if ( ( deltaX * deltaX + deltaY * deltaY ) < fProposedNeighborhoodRadiusSquared ):
											# if ( innerCount != referenceCount ):
										# for nNewSegmentInitialEndIndex in range( 2 ):
									# if ( not absoluteLineSegments[2] ):
								# for innerCount in range( nAbsoluteLineSegmentTotal ):
							# for nReferenceEndIndex in range( 2 ):
							
							# At last we've looked at all the candidate segment ends, as related to all the reference ends
							if ( not bFoundSegmentToAdd ):
								# This segment is solitary.
								# Must start a new line, not joined to any previous paths
								deltaX = absoluteLineSegments[referenceCount][1][0] - absoluteLineSegments[referenceCount][0][0]	# end minus start, in original direction
								deltaY = absoluteLineSegments[referenceCount][1][1] - absoluteLineSegments[referenceCount][0][1]	# end minus start, in original direction
								path += ( 'M %f,%f l %f,%f ' %
									( absoluteLineSegments[referenceCount][0][0], absoluteLineSegments[referenceCount][0][1],
									deltaX, deltaY ) )	# delta is from initial point
								fDistanceMovedWithPenUp += math.hypot(
									absoluteLineSegments[referenceCount][0][0] - ptLastPositionAbsolute[0],
									absoluteLineSegments[referenceCount][0][1] - ptLastPositionAbsolute[1] )
								ptLastPositionAbsolute[0] = absoluteLineSegments[referenceCount][0][0] + deltaX
								ptLastPositionAbsolute[1] = absoluteLineSegments[referenceCount][0][1] + deltaY
								absoluteLineSegments[ referenceCount ][2] = True			# True flags that this line segment has been
																							# added to the path to be drawn, so should
																							# no longer be a candidate for any kind of move.
								nPenLifts += 1
							else:						# if ( not bFoundSegmentToAdd ):
								# Found segment to add, and we must get to it in absolute terms
								deltaX = ( absoluteLineSegments[referenceCount][nReferenceEndIndexAtClosest][0] -
										absoluteLineSegments[referenceCount][not nReferenceEndIndexAtClosest][0] )
								# final point (which was closer to the closest continuation segment) minus initial point = deltaX

								deltaY = ( absoluteLineSegments[referenceCount][nReferenceEndIndexAtClosest][1] -
										absoluteLineSegments[referenceCount][not nReferenceEndIndexAtClosest][1] )
								# final point (which was closer to the closest continuation segment) minus initial point = deltaY
								
								path += ( 'M %f,%f l ' % (
									absoluteLineSegments[referenceCount][ not nReferenceEndIndexAtClosest][0],
									absoluteLineSegments[referenceCount][ not nReferenceEndIndexAtClosest][1] ) )
								fDistanceMovedWithPenUp += math.hypot(
									absoluteLineSegments[referenceCount][ not nReferenceEndIndexAtClosest][0] - ptLastPositionAbsolute[0],
									absoluteLineSegments[referenceCount][ not nReferenceEndIndexAtClosest][1] - ptLastPositionAbsolute[1] )
								ptLastPositionAbsolute[0] = absoluteLineSegments[referenceCount][ not nReferenceEndIndexAtClosest][0]
								ptLastPositionAbsolute[1] = absoluteLineSegments[referenceCount][ not nReferenceEndIndexAtClosest][1]
								# Note that this does not complete the line, as the completion (the deltaX, deltaY part) is being held in abeyance
								
								# We are coming up on a problem:
								# If we add a curve to the end of the line, we have made the curve extend beyond the end of the line,
								# and thus beyond the boundaries we should be respecting.
								# The solution is to hold in abeyance the actual plotting of the line,
								# holding it available for shrinking if a curve is to be added.
								# That is 
								relativePositionOfLastPlottedLineWasHeldInAbeyance = {}
								relativePositionOfLastPlottedLineWasHeldInAbeyance[0] = deltaX			# delta is from initial point
								relativePositionOfLastPlottedLineWasHeldInAbeyance[1] = deltaY			# Will be printed after we know if it must be modified
																										# to keep the ending join within bounds
								ptLastPositionAbsolute[0] += deltaX
								ptLastPositionAbsolute[1] += deltaY
																			
								absoluteLineSegments[ referenceCount ][2] = True			# True flags that this line segment has been
																							# added to the path to be drawn, so should
																							# no longer be a candidate for any kind of move.
								nPenLifts += 1
								# Now comes the speedup logic:
								# We've just drawn a segment starting at an absolute, not relative, position.
								# It was drawn from pt1 to pt2.
								# Look for an as-yet-not-drawn segment which has a beginning or ending
								# point "near" the end point of this absolute draw, and leave the pen down
								# while moving to and then drawing this found line.
								# Do this recursively, marking each segment True to show that
								# it has been "drawn" already.
								# pt2 is the reference point, ie. the point from which the next segment will start
								path = self.recursivelyAppendNearbySegmentIfAny(
								transformedHatchSpacing,
								0,
								referenceCount,
								nReferenceEndIndexAtClosest,
								nAbsoluteLineSegmentTotal,
								absoluteLineSegments,
								path,
								relativePositionOfLastPlottedLineWasHeldInAbeyance )
							# if ( not bFoundSegmentToAdd ): else:
						# if ( not absoluteLineSegments[referenceCount][2] ):	
					# while ( self.IndexOfNearestSegmentToLastPosition() ):
					self.joinFillsWithNode( key, stroke_width, path[:-1] )
				# if not self.options.reducePenLifts: else:
			# for key in self.hatches:

			'''
			if self.options.reducePenLifts:
				if ( nAbsoluteLineSegmentTotal != 0 ):
					inkex.errormsg( ' Saved %i%% of %i pen lifts.' % ( 100 * ( nAbsoluteLineSegmentTotal - nPenLifts ) / nAbsoluteLineSegmentTotal, nAbsoluteLineSegmentTotal ) )
					inkex.errormsg( ' pen lifts=%i, line segments=%i' % ( nPenLifts, nAbsoluteLineSegmentTotal ) )
				else:
					inkex.errormsg( ' No lines were plotted' )
					
				inkex.errormsg( ' Press OK' )
			# if self.options.reducePenLifts:
			#inkex.errormsg("Elapsed CPU time was %f" % (time.clock()-self.t0))
			'''
		else:	# if bHaveGrid:
			inkex.errormsg( ' Nothing to plot' )
		# if bHaveGrid: else:
	# def effect( self ):
	
	def recursivelyAppendNearbySegmentIfAny( 
		self,
		transformedHatchSpacing,
		nRecursionCount,
		nReferenceSegmentCount, 
		nReferenceEndIndex, 
		nAbsoluteLineSegmentTotal, 
		absoluteLineSegments, 
		cumulativePath, 
		relativePositionOfLastPlottedLineWasHeldInAbeyance ):
	
		global ptLastPositionAbsolute
		fProposedNeighborhoodRadiusSquared = self.ProposeNeighborhoodRadiusSquared( transformedHatchSpacing )
			
		# Look through all possibilities to choose the closest
		bFoundSegmentToAdd = False										# default assumption
		nNewSegmentInitialEndIndexAtClosest = 0
		nOuterCountAtClosest = -1
		fClosestDistanceSquared = 123456789.0							# just a random large number

		ptReference = absoluteLineSegments[nReferenceSegmentCount][nReferenceEndIndex]
		ptReferenceOtherEnd = absoluteLineSegments[nReferenceSegmentCount][not nReferenceEndIndex]
		fReferenceDeltaX = ptReferenceOtherEnd[0] - ptReference[0]
		fReferenceDeltaY = ptReferenceOtherEnd[1] - ptReference[1]
		fReferenceDirectionRadians = math.atan2( fReferenceDeltaY, fReferenceDeltaX )		# from other end to this end

		for outerCount in range( nAbsoluteLineSegmentTotal ):	# investigate all segments
			if ( not absoluteLineSegments[outerCount][2] ):
				# This segment currently undrawn, so it is a candidate for a path extension
				
				# Need to check both ends of each and every proposed segment until we find one in the neighborhood
				# Defines pt2 in the reference as the end which we want to extend
				
				for nNewSegmentInitialEndIndex in range( 2 ):
					# First try initial end of test segment (aka pt1) vs final end (aka pt2) of reference segment
					if ( outerCount != nReferenceSegmentCount ):	# don't investigate self ends
						deltaX = absoluteLineSegments[outerCount][nNewSegmentInitialEndIndex][0] - ptReference[0]		# proposed initial pt1 X minus existing final pt1 X
						deltaY = absoluteLineSegments[outerCount][nNewSegmentInitialEndIndex][1] - ptReference[1]		# proposed initial pt1 Y minus existing final pt1 Y
						if 	( ( deltaX * deltaX + deltaY * deltaY ) < fProposedNeighborhoodRadiusSquared ):
							fThisDistanceSquared = deltaX * deltaX + deltaY * deltaY
							ptNewSegmentThisEnd = absoluteLineSegments[outerCount][nNewSegmentInitialEndIndex]
							ptNewSegmentOtherEnd = absoluteLineSegments[outerCount][not nNewSegmentInitialEndIndex]
							fNewSegmentDeltaX = ptNewSegmentThisEnd[0] - ptNewSegmentOtherEnd[0]
							fNewSegmentDeltaY = ptNewSegmentThisEnd[1] - ptNewSegmentOtherEnd[1]
							fNewSegmentDirectionRadians = math.atan2( fNewSegmentDeltaY, fNewSegmentDeltaX )		# from other end to this end
							if ( not self.WouldBeAnAlternatingDirection( fReferenceDirectionRadians, fNewSegmentDirectionRadians ) ):
								# If this end would cause an alternating direction,
								# then exclude it regardless of how close it is
								pass
								# if ( not self.WouldBeAnAlternatingDirection( fReferenceDirectionRadians, fNewSegmentDirectionRadians ) ):
								
							elif ( fThisDistanceSquared < fClosestDistanceSquared ):
								# One other thing could rule out choosing this segment end:
								# Want to screen and remove two segments that, while close enough,
								# should be disqualified because they are colinear.  The reason for this is that
								# if they are colinear, they arose from the same global grid line, which means
								# that the gap between them arises from intersections with the boundary.
								# The idea here is that, all things being more-or-less equal,
								# we would like to give preference to connecting to a segment
								# which is the reverse of our current direction.  This makes for better
								# bezier curve join.
								# The criterion for being colinear is that the reference segment angle is effectively
								# the same as the line connecting the reference segment to the end of the new segment.

								fJoinerDirectionRadians = math.atan2( ptNewSegmentThisEnd[1] - ptReference[1], ptNewSegmentThisEnd[0] - ptReference[0] )
								if ( not self.AreCoLinear( fReferenceDirectionRadians, fJoinerDirectionRadians) ):
									# not colinear
									fClosestDistanceSquared = fThisDistanceSquared
									bFoundSegmentToAdd = True
									nNewSegmentInitialEndIndexAtClosest = nNewSegmentInitialEndIndex
									nOuterCountAtClosest = outerCount
									deltaXAtClosest = deltaX
									deltaYAtClosest = deltaY
								# if ( not self.AreCoLinear( fReferenceDirectionRadians, fJoinerDirectionRadians) ):
							# if ( not self.WouldBeAnAlternatingDirection( fReferenceDirectionRadians, fNewSegmentDirectionRadians ) ): elif ( fThisDistanceSquared < fClosestDistanceSquared ):
						# if 	( ( deltaX * deltaX + deltaY * deltaY ) < fProposedNeighborhoodRadiusSquared ):
					# if ( outerCount != nReferenceSegmentCount ):
				# for nNewSegmentInitialEndIndex in range( 2 ):
			# if ( not absoluteLineSegments[2] ):
		# for outerCount in range( nAbsoluteLineSegmentTotal ):

		# At last we've looked at all the candidate segment ends
		nRecursionCount += 1
		if ( ( not bFoundSegmentToAdd ) or ( ( nRecursionCount >= RECURSION_LIMIT ) ) ):
			cumulativePath += '%f,%f ' % ( relativePositionOfLastPlottedLineWasHeldInAbeyance[0], relativePositionOfLastPlottedLineWasHeldInAbeyance[1] )	# close out this segment
			ptLastPositionAbsolute[0] += relativePositionOfLastPlottedLineWasHeldInAbeyance[0]
			ptLastPositionAbsolute[1] += relativePositionOfLastPlottedLineWasHeldInAbeyance[1]
			return cumulativePath		# No undrawn segments were suitable for appending,
										# or there were so many that we worry about python recursion limit
		else:	# if ( not bFoundSegmentToAdd ):
			nNewSegmentInitialEndIndex = nNewSegmentInitialEndIndexAtClosest
			nNewSegmentFinalEndIndex = not nNewSegmentInitialEndIndex
			# nNewSegmentInitialEndIndex is 0 for connecting to pt1, 
			# and is 1 for connecting to pt2
			count = nOuterCountAtClosest			# count is the index of the segment to be appended.
			deltaX = deltaXAtClosest				# delta from final end of incoming segment to initial end of outgoing segment
			deltaY = deltaYAtClosest
			
			# First, move pen to initial end (may be either its pt1 or its pt2) of new segment
			
			# Insert a bezier curve for this transition element
			# To accomplish this, we need information on the incoming and outgoing segments.
			# Specifically, we need to know the lengths and angles of the segments in
			# order to decide on control points.
			fIncomingDeltaX = absoluteLineSegments[nReferenceSegmentCount][nReferenceEndIndex][0] - absoluteLineSegments[nReferenceSegmentCount][not nReferenceEndIndex][0]
			fIncomingDeltaY = absoluteLineSegments[nReferenceSegmentCount][nReferenceEndIndex][1] - absoluteLineSegments[nReferenceSegmentCount][not nReferenceEndIndex][1]
			# The outgoing deltas are based on the reverse direction of the segment, i.e. the segment pointing back to the joiner bezier curve
			fOutgoingDeltaX = absoluteLineSegments[count][nNewSegmentInitialEndIndex][0] - absoluteLineSegments[count][nNewSegmentFinalEndIndex][0]		# index is [count][start point = 0, final point = 1][0=x, 1=y]
			fOutgoingDeltaY = absoluteLineSegments[count][nNewSegmentInitialEndIndex][1] - absoluteLineSegments[count][nNewSegmentFinalEndIndex][1]
			
			lengthOfIncoming = math.hypot( fIncomingDeltaX, fIncomingDeltaY )
			lengthOfOutgoing = math.hypot( fOutgoingDeltaX, fOutgoingDeltaY )
			
			# We are going to trim-up the ends of the incoming and outgoing segments,
			# in order to get a curve which reliably does not extend beyond the boundary.
			# Crude readings from inkscape on bezier curve overshoot, using control points extended hatch-spacing distance parallel to segment:
			# when end points are in line, overshoot 12/16 in direction of segment
			#		   when at 45 degrees, overshoot 12/16 in direction of segment
			#		   when at 60 degrees, overshoot 12/16 in direction of segment
			# Conclusion, at any angle, remove 0.75 * hatch spacing from the length of both lines, 
			# where 0.75 is, by no coincidence, BEZIER_OVERSHOOT_MULTIPLIER
			
			# If hatches are getting quite short, we can use a smaller Bezier loop at
			# the end to squeeze into smaller spaces.  We'll use a normal nice smooth
			# curve for non-short hatches
			fDesiredShortenForSmoothestJoin = transformedHatchSpacing * BEZIER_OVERSHOOT_MULTIPLIER		# This is what we really want to use for smooth curves
			# Separately check incoming vs outgoing lengths to see if bezier distances must be reduced,
			# then choose greatest reduction to apply to both - lest we go off-course
			# Finally, clip reduction to be no less than 1.0
			fControlPointDividerIncoming = 2.0 * fDesiredShortenForSmoothestJoin / lengthOfIncoming
			fControlPointDividerOutgoing = 2.0 * fDesiredShortenForSmoothestJoin / lengthOfOutgoing
			if ( fControlPointDividerIncoming > fControlPointDividerOutgoing ):
				fLargestDesiredControlPointDivider = fControlPointDividerIncoming
			else:	# if ( fControlPointDividerIncoming > fControlPointDividerOutgoing ):
				fLargestDesiredControlPointDivider = fControlPointDividerOutgoing
			# if ( fControlPointDividerIncoming > fControlPointDividerOutgoing ): else:
			if (fLargestDesiredControlPointDivider < 1.0):
				fControlPointDivider = 1.0
			else:	# if (fLargestDesiredControlPointDivider < 1.0):
				fControlPointDivider = fLargestDesiredControlPointDivider
			# if (fLargestDesiredControlPointDivider < 1.0): else:
			fDesiredShorten = fDesiredShortenForSmoothestJoin / fControlPointDivider

			ptDeltaToSubtractFromIncomingEnd = self.RelativeControlPointPosition( fDesiredShorten, fIncomingDeltaX, fIncomingDeltaY, 0, 0 )
				# Note that this will be subtracted from the _point held in abeyance_.
			relativePositionOfLastPlottedLineWasHeldInAbeyance[0] -= ptDeltaToSubtractFromIncomingEnd[0]
			relativePositionOfLastPlottedLineWasHeldInAbeyance[1] -= ptDeltaToSubtractFromIncomingEnd[1]
			
			ptDeltaToAddToOutgoingStart = self.RelativeControlPointPosition( fDesiredShorten, fOutgoingDeltaX, fOutgoingDeltaY, 0, 0 )
			
			# We know that when we tack on a curve, we must chop some off the end of the incoming segment,
			# and also chop some off the start of the outgoing segment.
			# Now, we know we want the control points to be on a projection of each segment,
			# in order that there be no abrupt change of plotting angle.  The question is, how
			# far beyond the endpoint should we place the control point.
			ptRelativeControlPointIncoming = self.RelativeControlPointPosition(
												transformedHatchSpacing / fControlPointDivider,
												fIncomingDeltaX,
												fIncomingDeltaY,
												0,
												0 )
			ptRelativeControlPointOutgoing = self.RelativeControlPointPosition(
												transformedHatchSpacing / fControlPointDivider,
												fOutgoingDeltaX,
												fOutgoingDeltaY,
												deltaX,
												deltaY)
												
			cumulativePath += '%f,%f ' % ( relativePositionOfLastPlottedLineWasHeldInAbeyance[0], relativePositionOfLastPlottedLineWasHeldInAbeyance[1] )	# close out this segment, which has been modified
			ptLastPositionAbsolute[0] += relativePositionOfLastPlottedLineWasHeldInAbeyance[0]
			ptLastPositionAbsolute[1] += relativePositionOfLastPlottedLineWasHeldInAbeyance[1]
			# add bezier cubic curve
			cumulativePath += ( 'c %f,%f %f,%f %f,%f l ' %
				( ptRelativeControlPointIncoming[0],
				ptRelativeControlPointIncoming[1],
				ptRelativeControlPointOutgoing[0],
				ptRelativeControlPointOutgoing[1],
				deltaX,
				deltaY ) )
			ptLastPositionAbsolute[0] += deltaX
			ptLastPositionAbsolute[1] += deltaY
			# Next, move pen in appropriate direction to draw the new segment, given that
			# we have just moved to the initial end of the new segment.
			# This needs special treatment, as we just did some length changing.
			deltaX = absoluteLineSegments[count][nNewSegmentFinalEndIndex][0] - absoluteLineSegments[count][nNewSegmentInitialEndIndex][0] + ptDeltaToAddToOutgoingStart[0]
			deltaY = absoluteLineSegments[count][nNewSegmentFinalEndIndex][1] - absoluteLineSegments[count][nNewSegmentInitialEndIndex][1] + ptDeltaToAddToOutgoingStart[1]
			relativePositionOfLastPlottedLineWasHeldInAbeyance[0] = deltaX			# delta is from initial point
			relativePositionOfLastPlottedLineWasHeldInAbeyance[1] = deltaY			# Will be printed after we know if it must be modified
			
			# Mark this segment as drawn
			absoluteLineSegments[count][2] = True

			cumulativePath = self.recursivelyAppendNearbySegmentIfAny( transformedHatchSpacing, nRecursionCount, count, nNewSegmentFinalEndIndex, nAbsoluteLineSegmentTotal, absoluteLineSegments, cumulativePath, relativePositionOfLastPlottedLineWasHeldInAbeyance )
			return cumulativePath
		# if ( not bFoundSegmentToAdd ): else:
	# def recursivelyAppendNearbySegmentIfAny( ... ):
			
	def ProposeNeighborhoodRadiusSquared( self, transformedHatchSpacing ):
		return transformedHatchSpacing * transformedHatchSpacing * self.options.hatchScope * self.options.hatchScope
			# The multiplier of x generates a radius of x^0.5 times the hatch spacing.
			
	def RelativeControlPointPosition( self, distance, fDeltaX, fDeltaY, deltaX, deltaY ):
	
		# returns the point, relative to 0, 0 offset by deltaX, deltaY,
		# which extends a distance of "distance" at a slope defined by fDeltaX and fDeltaY
		ptReturn = [0, 0]
		
		if ( fDeltaX == 0 ):
			ptReturn[0] = deltaX
			ptReturn[1] = math.copysign( distance, fDeltaY ) + deltaY
		elif ( fDeltaY == 0 ):
			ptReturn[0] = math.copysign( distance, fDeltaX ) + deltaX
			ptReturn[1] = deltaY
		else:
			fSlope = math.atan2( fDeltaY, fDeltaX )
			ptReturn[0] = distance * math.cos( fSlope ) + deltaX
			ptReturn[1] = distance * math.sin( fSlope ) + deltaY

		return ptReturn

	def WouldBeAnAlternatingDirection( self, fReferenceDirectionRadians, fNewSegmentDirectionRadians ):
		# atan2 returns values in the range -pi to +pi, so we must evaluate difference values
		# in the range of -2*pi to +2*pi
		fDirectionDifferenceRadians = fReferenceDirectionRadians - fNewSegmentDirectionRadians
		if ( fDirectionDifferenceRadians < 0 ):
			fDirectionDifferenceRadians += 2 * math.pi
		# Without having changed the vector direction of the difference, we have
		# now reduced the range to 0 to 2*pi
		fDirectionDifferenceRadians -= math.pi		# flip opposite direction to coincide with same direction
		# Of course they may not be _exactly_ pi different due to osmosis, so allow a tolerance
		bRetVal = ( abs(fDirectionDifferenceRadians) < RADIAN_TOLERANCE_FOR_ALTERNATING_DIRECTION )
		
		return	bRetVal
		
	def AreCoLinear( self, fDirection1Radians, fDirection2Radians ):
		# allow slight difference in angles, for floating-point indeterminacy
		fAbsDeltaRadians = abs( fDirection1Radians - fDirection2Radians )
		if ( ( fAbsDeltaRadians < RADIAN_TOLERANCE_FOR_COLINEAR )  ):
			return True
		elif ( ( abs( fAbsDeltaRadians - math.pi ) < RADIAN_TOLERANCE_FOR_COLINEAR )  ):
			return True
		else:
			return False

if __name__ == '__main__':

	e = Eggbot_Hatch()
	e.affect()
