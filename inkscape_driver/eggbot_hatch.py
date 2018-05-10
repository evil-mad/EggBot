#!/usr/bin/env python
# coding=utf-8

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

# Updated by Nathan Depew, 12/6/2017
# Modified hatch fill to create hatches as a relevant object it found on the SVG tree
# This prevents extremely complex plots from generating glitches
# Modifications are limited to recursivelyTraverseSvg and effect methods 

#
# Current software version:
# (v2.1.0, December 6, 2017)
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

import math

import bezmisc
import cspsubdiv
import cubicsuperpath
import inkex
import plot_utils  # https://github.com/evil-mad/plotink
import simplepath
import simplestyle
from simpletransform import applyTransformToPath, applyTransformToPoint, composeTransform, parseTransform

N_PAGE_WIDTH = 3200
N_PAGE_HEIGHT = 800

F_MINGAP_SMALL_VALUE = 0.0000000001  
# Was 0.00001 in the original version which did not have joined lines.
# Reducing this by a factor of 10^5 decreased probability of occurrence of
# the bug in the original, which got confused when the path barely
# grazed a corner.

BEZIER_OVERSHOOT_MULTIPLIER = 0.75  # evaluation of cubic Bezier curve equation value,
# at x = 0, with
# endpoints at ( -0.5, 0 ), ( +0.5, 0 )
# and control points at ( -0.5, 1.0 ), ( +0.5, 1.0 )

RADIAN_TOLERANCE_FOR_COLINEAR = 0.1  
# Pragmatically adjusted to allow adjacent segments from the same scan line, even short ones,
# to be classified as having the same angle

RADIAN_TOLERANCE_FOR_ALTERNATING_DIRECTION = 0.1  
# Pragmatic adjustment again, as with colinearity tolerance

RECURSION_LIMIT = 500  
# Pragmatic - if too high, risk runtime python error; 
# if too low, miss some chances for reducing pen lifts

EXTREME_POS = 1.0E70 # Extremely large positive number
EXTREME_NEG = -1.0E70 # Extremely large negative number

MIN_HATCH_FRACTION = 0.25  
# Minimum hatch length, as a fraction of the hatch spacing.

"""
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
"""


def intersect(p1, p2, p3, p4):
    """
    Determine if two line segments defined by the four points p1 & p2 and
    p3 & p4 intersect.  If they do intersect, then return the fractional
    point of intersection "sa" along the first line at which the
    intersection occurs.
    """

    # Precompute these values -- note that we're basically shifting from
    #
    #        p = p1 + s (p2 - p1)
    #
    # to
    #
    #         p = p1 + s d
    #
    # where D is a direction vector.  The solution remains the same of
    # course.  We'll just be computing D once for each line rather than
    # computing it a couple of times.

    d21x = p2[0] - p1[0]
    d21y = p2[1] - p1[1]
    d43x = p4[0] - p3[0]
    d43y = p4[1] - p3[1]

    # Denominator
    d = d21x * d43y - d21y * d43x

    # Return now if the denominator is zero
    if d == 0:
        return -1.0

    # For our purposes, the first line segment given
    # by p1 & p2 is the LONG hatch line running through
    # the entire drawing.  And, p3 & p4 describe the
    # usually much shorter line segment from a polygon.
    # As such, we compute sb first as it's more likely
    # to indicate "no intersection".  That is, sa is
    # more likely to indicate an intersection with a
    # much a long line containing p3 & p4.

    nb = (p1[1] - p3[1]) * d21x - (p1[0] - p3[0]) * d21y

    # Could first check if abs(nb) > abs(d) or if
    # the signs differ.
    sb = float(nb) / float(d)
    if sb < 0 or sb > 1:
        return -1.0

    na = (p1[1] - p3[1]) * d43x - (p1[0] - p3[0]) * d43y
    sa = float(na) / float(d)
    if sa < 0 or sa > 1:
        return -1.0

    return sa


def interstices(self, p1, p2, paths, hatches, b_hold_back_hatches, f_hold_back_steps):
    """
    For the line L defined by the points p1 & p2, determine the segments
    of L which lie within the polygons described by the paths stored in
    "paths"

    p1 -- (x,y) coordinate [list]
    p2 -- (x,y) coordinate [list]
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
    """

    d_and_a = []
    # p1 & p2 is the hatch line
    # p3 & p4 is the polygon edge to check
    for path in paths:
        for subpath in paths[path]:
            p3 = subpath[0]
            for p4 in subpath[1:]:
                s = intersect(p1, p2, p3, p4)
                if 0.0 <= s <= 1.0:
                    # Save this intersection point along the hatch line
                    if b_hold_back_hatches:
                        # We will need to know how the hatch meets the polygon segment, so that we can
                        # calculate the end of a shorter line that stops short
                        # of the polygon segment.
                        # We compute the angle now while we have the information required,
                        # but do _not_ apply it now, as we need the real,original, intersects
                        # for the odd/even inside/outside operations yet to come.
                        # Note that though the intersect() routine _could_ compute the join angle,
                        # we do it here because we go thru here much less often than we go thru intersect().
                        angle_hatch_radians = math.atan2(-(p2[1] - p1[1]), (p2[0] - p1[0]))  # from p1 toward p2, cartesian coordinates
                        angle_segment_radians = math.atan2(-(p4[1] - p3[1]), (p4[0] - p3[0]))  # from p3 toward p4, cartesian coordinates
                        angle_difference_radians = angle_hatch_radians - angle_segment_radians
                        # coerce to range -pi to +pi
                        if angle_difference_radians > math.pi:
                            angle_difference_radians -= 2 * math.pi
                        elif angle_difference_radians < -math.pi:
                            angle_difference_radians += 2 * math.pi
                        f_sin_of_join_angle = math.sin(angle_difference_radians)
                        f_abs_sin_of_join_angle = abs(f_sin_of_join_angle)
                        if f_abs_sin_of_join_angle != 0.0:  # Worrying about case of intersecting a segment parallel to the hatch
                            prelim_length_to_be_removed = f_hold_back_steps / f_abs_sin_of_join_angle
                            b_unconditionally_excise_hatch = False
                        else:
                            b_unconditionally_excise_hatch = True

                        if not b_unconditionally_excise_hatch:
                            # The relevant end of the segment is the end from which the hatch approaches at an acute angle.
                            intersection = [0, 0]
                            intersection[0] = p1[0] + s * (p2[0] - p1[0])  # compute intersection point of hatch with segment
                            intersection[1] = p1[1] + s * (p2[1] - p1[1])  # intersecting hatch line starts at p1, vectored toward p2,
                            # but terminates at intersection
                            # Note that atan2 returns answer in range -pi to pi
                            # Which end is the approach end of the hatch to the segment?
                            # The dot product tells the answer:
                            #    if dot product is positive, p2 is at the p4 end,
                            #    else p2 is at the p3 end
                            # We really don't need to take the time to actually take
                            #     the cosine of the angle, we are just interested in
                            #    the quadrant within which the angle lies.
                            # I'm sure there is an elegant way to do this, but I'll settle for results just now.
                            # If the angle is in quadrants I or IV then p4 is the relevant end, otherwise p3 is
                            # nb: Y increases down, rather than up
                            # nb: difference angle has been forced to the range -pi to +pi
                            if abs(angle_difference_radians) < math.pi / 2:
                                # It's near the p3 the relevant end from which the hatch departs
                                dist_intersection_to_relevant_end = math.hypot(p3[0] - intersection[0], p3[1] - intersection[1])
                                dist_intersection_to_irrelevant_end = math.hypot(p4[0] - intersection[0], p4[1] - intersection[1])
                            else:
                                # It's near the p4 end from which the hatch departs
                                dist_intersection_to_relevant_end = math.hypot(p4[0] - intersection[0], p4[1] - intersection[1])
                                dist_intersection_to_irrelevant_end = math.hypot(p3[0] - intersection[0], p3[1] - intersection[1])

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
                            length_remove_starting_hatch = prelim_length_to_be_removed
                            length_remove_ending_hatch = prelim_length_to_be_removed

                            # Now check each of the two ends
                            if prelim_length_to_be_removed > (dist_intersection_to_relevant_end + f_hold_back_steps):
                                # Yes, would be excessive holdback approaching from this direction
                                length_remove_starting_hatch = dist_intersection_to_relevant_end + f_hold_back_steps
                            if prelim_length_to_be_removed > (dist_intersection_to_irrelevant_end + f_hold_back_steps):
                                # Yes, would be excessive holdback approaching from other direction
                                length_remove_ending_hatch = dist_intersection_to_irrelevant_end + f_hold_back_steps

                            d_and_a.append((s, path, length_remove_starting_hatch, length_remove_ending_hatch))
                        else:
                            d_and_a.append((s, path, 123456.0, 123456.0))  # Mark for complete hatch excision, hatch is parallel to segment
                            # Just a random number guaranteed large enough to be longer than any hatch length
                    else:
                        d_and_a.append((s, path, 0, 0))  # zero length to be removed from hatch

                p3 = p4

    # Return now if there were no intersections
    if len(d_and_a) == 0:
        return None

    d_and_a.sort()

    # Remove duplicate intersections.  A common case where these arise
    # is when the hatch line passes through a vertex where one line segment
    # ends and the next one begins.

    # Having sorted the data, it's trivial to just scan through
    # removing duplicates as we go and then truncating the array

    n = len(d_and_a)
    i_last = 1
    i = 1
    last = d_and_a[0]
    while i < n:
        if (abs(d_and_a[i][0] - last[0])) > F_MINGAP_SMALL_VALUE:
            d_and_a[i_last] = last = d_and_a[i]
            i_last += 1
        i += 1
    d_and_a = d_and_a[:i_last]
    if len(d_and_a) < 2:
        return

    # Now, entries with even valued indices into sa[] are where we start
    # a hatch line and odd valued indices where we end the hatch line.

    i = 0
    while i < (len(d_and_a) - 1):
        if d_and_a[i][1] not in hatches:
            hatches[d_and_a[i][1]] = []

        x1 = p1[0] + d_and_a[i][0] * (p2[0] - p1[0])
        y1 = p1[1] + d_and_a[i][0] * (p2[1] - p1[1])
        x2 = p1[0] + d_and_a[i + 1][0] * (p2[0] - p1[0])
        y2 = p1[1] + d_and_a[i + 1][0] * (p2[1] - p1[1])

        # These are the hatch ends if we are _not_ holding off from the boundary.
        if not b_hold_back_hatches:
            hatches[d_and_a[i][1]].append([[x1, y1], [x2, y2]])
        else:
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
            #     Y / cutlength = sin(angle)
            # therefore:
            #   cutlength = Y / sin(angle)
            # Fortunately, we have already stored this angle for exactly this purpose.
            # For each end, trim back the hatch line by the amount required by
            # its own angle.  If the resultant diminished hatch is too short,
            # remove it from consideration by marking it as already drawn - a
            # fiction, but is much quicker than actually removing the hatch from the list.

            f_min_allowed_hatch_length = self.options.hatchSpacing * MIN_HATCH_FRACTION
            f_initial_hatch_length = math.hypot(x2 - x1, y2 - y1)
            # We did as much as possible of the inset operation back when we were finding intersections.
            # We did it back then because at that point we knew more about the geometry than we know now.
            # Now we don't know where the ends of the segments are, so we can't address issue 22 here.
            f_length_to_be_removed_from_pt1 = d_and_a[i][3]
            f_length_to_be_removed_from_pt2 = d_and_a[i + 1][2]

            if (f_initial_hatch_length - (f_length_to_be_removed_from_pt1 + f_length_to_be_removed_from_pt2)) <= f_min_allowed_hatch_length:
                pass  # Just don't insert it into the hatch list
            else:
                """
                Use:
                def RelativeControlPointPosition( self, distance, fDeltaX, fDeltaY, deltaX, deltaY ):
                    # returns the point, relative to 0, 0 offset by deltaX, deltaY,
                    # which extends a distance of "distance" at a slope defined by fDeltaX and fDeltaY
                """
                pt1 = self.RelativeControlPointPosition(f_length_to_be_removed_from_pt1, x2 - x1, y2 - y1, x1, y1)
                pt2 = self.RelativeControlPointPosition(f_length_to_be_removed_from_pt2, x1 - x2, y1 - y2, x2, y2)
                hatches[d_and_a[i][1]].append([[pt1[0], pt1[1]], [pt2[0], pt2[1]]])

        # Remember the relative start and end of this hatch segment
        last_d_and_a = [d_and_a[i], d_and_a[i + 1]]

        i += 2


def inverseTransform(tran):
    """
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

    To invert the transform stored Inkscape style, we wish to
    produce

        [[d/D, -c/D, (cf - de)/D], [-b/D, a/D, (be-af)/D]]

    where

        D = 1 / (ad - bc)
    """

    D = tran[0][0] * tran[1][1] - tran[1][0] * tran[0][1]
    if D == 0:
        return None

    return [[tran[1][1] / D, -tran[0][1] / D,
             (tran[0][1] * tran[1][2] - tran[1][1] * tran[0][2]) / D],
            [-tran[1][0] / D, tran[0][0] / D,
             (tran[1][0] * tran[0][2] - tran[0][0] * tran[1][2]) / D]]


def subdivideCubicPath(sp, flat, i=1):
    """
    Break up a bezier curve into smaller curves, each of which
    is approximately a straight line within a given tolerance
    (the "smoothness" defined by [flat]).

    This is a modified version of cspsubdiv.cspsubdiv() rewritten
    to avoid recurrence.
    """

    while True:
        while True:
            if i >= len(sp):
                return

            p0 = sp[i - 1][1]
            p1 = sp[i - 1][2]
            p2 = sp[i][0]
            p3 = sp[i][1]

            b = (p0, p1, p2, p3)

            if cspsubdiv.maxdist(b) > flat:
                break

            i += 1

        one, two = bezmisc.beziersplitatt(b, 0.5)
        sp[i - 1][2] = one[1]
        sp[i][0] = two[2]
        p = [one[2], one[3], two[1]]
        sp[i:1] = [p]


def distanceSquared(p1, p2):
    """
    Pythagorean distance formula WITHOUT the square root.  Since
    we just want to know if the distance is less than some fixed
    fudge factor, we can just square the fudge factor once and run
    with it rather than compute square roots over and over.
    """

    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    return dx * dx + dy * dy


class Eggbot_Hatch(inkex.Effect):

    def __init__(self):

        inkex.Effect.__init__(self)

        self.xmin, self.ymin = (0.0, 0.0)
        self.xmax, self.ymax = (0.0, 0.0)
        self.paths = {}
        self.grid = []
        self.hatches = {}
        self.transforms = {}

        # For handling an SVG viewbox attribute, we will need to know the
        # values of the document's <svg> width and height attributes as well
        # as establishing a transform from the viewbox to the display.
        self.docWidth = float(N_PAGE_WIDTH)
        self.docHeight = float(N_PAGE_HEIGHT)
        self.docTransform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]

        self.OptionParser.add_option(
                "--holdBackSteps", action="store", type="float",
                dest="holdBackSteps", default=3.0,
                help="How far hatch strokes stay from boundary (steps)")
        self.OptionParser.add_option(
                "--hatchScope", action="store", type="float",
                dest="hatchScope", default=3.0,
                help="Radius searched for segments to join (units of hatch width)")
        self.OptionParser.add_option(
                "--holdBackHatchFromEdges", action="store", dest="holdBackHatchFromEdges",
                type="inkbool", default=True,
                help="Stay away from edges, so no need for inset")
        self.OptionParser.add_option(
                "--reducePenLifts", action="store", dest="reducePenLifts",
                type="inkbool", default=True,
                help="Reduce plotting time by joining some hatches")
        self.OptionParser.add_option(
                "--crossHatch", action="store", dest="crossHatch",
                type="inkbool", default=False,
                help="Generate a cross hatch pattern")
        self.OptionParser.add_option(
                "--hatchAngle", action="store", type="float",
                dest="hatchAngle", default=90.0,
                help="Angle of inclination for hatch lines")
        self.OptionParser.add_option(
                "--hatchSpacing", action="store", type="float",
                dest="hatchSpacing", default=10.0,
                help="Spacing between hatch lines")
        self.OptionParser.add_option(
                "--tolerance", action="store", type="float",
                dest="tolerance", default=20.0,
                help="Allowed deviation from original paths")
        self.OptionParser.add_option("--tab",  # NOTE: value is not used.
                                     action="store", type="string", dest="tab", default="splash",
                                     help="The active tab when Apply was pressed")

    def getDocProps(self):

        """
        Get the document's height and width attributes from the <svg> tag.
        Use a default value in case the property is not present or is
        expressed in units of percentages.
        """

        self.docHeight = plot_utils.getLength(self, 'height', N_PAGE_HEIGHT)
        self.docWidth = plot_utils.getLength(self, 'width', N_PAGE_WIDTH)

        if self.docHeight is None or self.docWidth is None:
            return False
        else:
            return True

    def handleViewBox(self):

        """
        Set up the document-wide transform in the event that the document has an SVG viewbox
        """

        if self.getDocProps():
            viewbox = self.document.getroot().get('viewBox')
            if viewbox:
                vinfo = viewbox.strip().replace(',', ' ').split(' ')
                if vinfo[2] != 0 and vinfo[3] != 0:
                    sx = self.docWidth / float(vinfo[2])
                    sy = self.docHeight / float(vinfo[3])
                    self.docTransform = parseTransform('scale({0:f},{1:f})'.format(sx, sy))

    def addPathVertices(self, path, node=None, transform=None):

        """
        Decompose the path data from an SVG element into individual
        subpaths, each starting with an absolute move-to (x, y)
        coordinate followed by one or more absolute line-to (x, y)
        coordinates.  Each subpath is stored as a list of (x, y)
        coordinates, with the first entry understood to be a
        move-to coordinate and the rest line-to coordinates.  A list
        is then made of all the subpath lists and then stored in the
        self.paths dictionary using the path's lxml.etree node pointer
        as the dictionary key.
        """

        if not path or len(path) == 0:
            return

        # parsePath() may raise an exception.  This is okay
        sp = simplepath.parsePath(path)
        if not sp or len(sp) == 0:
            return

        # Get a cubic super duper path
        p = cubicsuperpath.CubicSuperPath(sp)
        if not p or len(p) == 0:
            return

        # Apply any transformation
        if transform is not None:
            applyTransformToPath(transform, p)

        # Now traverse the simplified path
        subpaths = []
        subpath_vertices = []
        for sp in p:
            # We've started a new subpath
            # See if there is a prior subpath and whether we should keep it
            if len(subpath_vertices):
                if distanceSquared(subpath_vertices[0], subpath_vertices[-1]) < 1:
                    # Keep the prior subpath: it appears to be a closed path
                    subpaths.append(subpath_vertices)
            subpath_vertices = []
            subdivideCubicPath(sp, float(self.options.tolerance / 100))
            for csp in sp:
                # Add this vertex to the list of vertices
                subpath_vertices.append(csp[1])

        # Handle final subpath
        if len(subpath_vertices):
            if distanceSquared(subpath_vertices[0], subpath_vertices[-1]) < 1:
                # Path appears to be closed so let's keep it
                subpaths.append(subpath_vertices)

        # Empty path?
        if len(subpaths) == 0:
            return

        # And add this path to our dictionary of paths
        self.paths[node] = subpaths

        # And save the transform for this element in a dictionary keyed
        # by the element's lxml node pointer
        self.transforms[node] = transform

    def getBoundingBox(self):

        """
        Determine the bounding box for our collection of polygons
        """

        self.xmin, self.xmax = EXTREME_POS, EXTREME_NEG
        self.ymin, self.ymax = EXTREME_POS, EXTREME_NEG
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

    def recursivelyTraverseSvg(self, a_node_list, mat_current=None, parent_visibility='visible'):
        """
        Recursively walk the SVG document, building polygon vertex lists
        for each graphical element we support.

        Rendered SVG elements:
            <circle>, <ellipse>, <line>, <path>, <polygon>, <polyline>, <rect>

        Supported SVG elements:
            <group>, <use>

        Ignored SVG elements:
            <defs>, <eggbot>, <metadata>, <namedview>, <pattern>

        All other SVG elements trigger an error (including <text>)

        Once a supported graphical element is found, we call functions to
        create a hatchfill specific to this element. These hatches and their
        corresponding transforms are stored in self.hatches and self.transforms
        These two dictionaries are used when we return to the effect method
        in joinFillsWithNode()

        """
        if mat_current is None:
            mat_current = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        for node in a_node_list:

            """
             Initialize dictionary for each new node
             This allows us to create hatch fills as if each 
             object to be hatched has been selected individually

            """
            self.xmin, self.ymin = (0.0, 0.0)
            self.xmax, self.ymax = (0.0, 0.0)
            self.paths = {}
            self.grid = []

            # Ignore invisible nodes
            v = node.get('visibility', parent_visibility)
            if v == 'inherit':
                v = parent_visibility
            if v == 'hidden' or v == 'collapse':
                pass

            # first apply the current matrix transform to this node's transform
            mat_new = composeTransform(mat_current, parseTransform(node.get("transform")))

            if node.tag in [inkex.addNS('g', 'svg'), 'g']:
                self.recursivelyTraverseSvg(node, mat_new, parent_visibility=v)

            elif node.tag in [inkex.addNS('use', 'svg'), 'use']:

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

                refid = node.get(inkex.addNS('href', 'xlink'))

                # [1:] to ignore leading '#' in reference
                path = '//*[@id="{0}"]'.format(refid[1:])
                refnode = node.xpath(path)
                if refnode:
                    x = float(node.get('x', '0'))
                    y = float(node.get('y', '0'))
                    # Note: the transform has already been applied
                    if x != 0 or y != 0:
                        mat_new2 = composeTransform(mat_new, parseTransform('translate({0:f},{1:f})'.format(x, y)))
                    else:
                        mat_new2 = mat_new
                    v = node.get('visibility', v)
                    self.recursivelyTraverseSvg(refnode, mat_new2, parent_visibility=v)

            elif node.tag == inkex.addNS('path', 'svg'):

                path_data = node.get('d')
                if path_data:
                    self.addPathVertices(path_data, node, mat_new)
                    # We now have a path we want to apply a (cross)hatch to
                    # Apply appropriate functions
                    b_have_grid = self.makeHatchGrid(float(self.options.hatchAngle), float(self.options.hatchSpacing), True)
                    if b_have_grid:
                        if self.options.crossHatch:
                            self.makeHatchGrid(float(self.options.hatchAngle + 90.0), float(self.options.hatchSpacing), False)
                        # Now loop over our hatch lines looking for intersections
                        for h in self.grid:
                            interstices(self, (h[0], h[1]), (h[2], h[3]), self.paths, self.hatches, self.options.holdBackHatchFromEdges, self.options.holdBackSteps)

            elif node.tag in [inkex.addNS('rect', 'svg'), 'rect']:

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
                x = float(node.get('x'))
                y = float(node.get('y'))

                w = float(node.get('width', '0'))
                h = float(node.get('height', '0'))
                a = [['M ', [x, y]],
                     [' l ', [w, 0]],
                     [' l ', [0, h]],
                     [' l ', [-w, 0]],
                     [' Z', []],
                     ]
                self.addPathVertices(simplepath.formatPath(a), node, mat_new)
                # We now have a path we want to apply a (cross)hatch to
                # Apply appropriate functions
                b_have_grid = self.makeHatchGrid(float(self.options.hatchAngle), float(self.options.hatchSpacing), True)
                if b_have_grid:
                    if self.options.crossHatch:
                        self.makeHatchGrid(float(self.options.hatchAngle + 90.0), float(self.options.hatchSpacing), False)
                        # Now loop over our hatch lines looking for intersections
                    for h in self.grid:
                        interstices(self, (h[0], h[1]), (h[2], h[3]), self.paths, self.hatches, self.options.holdBackHatchFromEdges, self.options.holdBackSteps)

            elif node.tag in [inkex.addNS('line', 'svg'), 'line']:

                # Convert
                #
                #   <line x1="X1" y1="Y1" x2="X2" y2="Y2/>
                #
                # to
                #
                #   <path d="MX1,Y1 LX2,Y2"/>

                x1 = float(node.get('x1'))
                y1 = float(node.get('y1'))
                x2 = float(node.get('x2'))
                y2 = float(node.get('y2'))

                a = [['M ', [x1, y1]],
                     [' L ', [x2, y2]],
                     ]
                self.addPathVertices(simplepath.formatPath(a), node, mat_new)
                # We now have a path we want to apply a (cross)hatch to
                # Apply appropriate functions
                b_have_grid = self.makeHatchGrid(float(self.options.hatchAngle), float(self.options.hatchSpacing), True)
                if b_have_grid:
                    if self.options.crossHatch:
                        self.makeHatchGrid(float(self.options.hatchAngle + 90.0), float(self.options.hatchSpacing), False)
                        # Now loop over our hatch lines looking for intersections
                    for h in self.grid:
                        interstices(self, (h[0], h[1]), (h[2], h[3]), self.paths, self.hatches, self.options.holdBackHatchFromEdges, self.options.holdBackSteps)

            elif node.tag in [inkex.addNS('polyline', 'svg'), 'polyline']:

                # Convert
                #
                #  <polyline points="x1,y1 x2,y2 x3,y3 [...]"/>
                #
                # to
                #
                #   <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...]"/>
                #
                # Note: we ignore polylines with no points

                pl = node.get('points', '').strip()
                if pl == '':
                    continue
                pa = pl.split()
                if not pa:
                    continue
                pathLength = len( pa )
                if (pathLength < 4): # Minimum of x1,y1 x2,y2 required.
                    continue

                d = "M " + pa[0] + " " + pa[1]
                i = 2
                while (i < (pathLength - 1 )):
                    d += " L " + pa[i] + " " + pa[i + 1]
                    i += 2

                if d:
                    self.addPathVertices(d, node, mat_new)

                    # We now have a path we want to apply a (cross)hatch to
                    # Apply appropriate functions
                    b_have_grid = self.makeHatchGrid(float(self.options.hatchAngle), float(self.options.hatchSpacing), True)
                    if b_have_grid:
                        if self.options.crossHatch:
                            self.makeHatchGrid(float(self.options.hatchAngle + 90.0), float(self.options.hatchSpacing), False)
                            # Now loop over our hatch lines looking for intersections
                        for h in self.grid:
                            interstices(self, (h[0], h[1]), (h[2], h[3]), self.paths, self.hatches, self.options.holdBackHatchFromEdges, self.options.holdBackSteps)

            elif node.tag in [inkex.addNS('polygon', 'svg'), 'polygon']:
                # Convert
                #
                #  <polygon points="x1,y1 x2,y2 x3,y3 [...]"/>
                #
                # to
                #
                #   <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...] Z"/>
                #
                # Note: we ignore polygons with no points

                pl = node.get('points', '').strip()

                pa = pl.split()
                d = "".join(["M " + pa[i] if i == 0 else " L " + pa[i] for i in range(0, len(pa))])
                d += " Z"
                self.addPathVertices(d, node, mat_new)
                # We now have a path we want to apply a (cross)hatch to
                # Apply appropriate functions
                b_have_grid = self.makeHatchGrid(float(self.options.hatchAngle), float(self.options.hatchSpacing), True)
                if b_have_grid:
                    if self.options.crossHatch:
                        self.makeHatchGrid(float(self.options.hatchAngle + 90.0), float(self.options.hatchSpacing), False)
                        # Now loop over our hatch lines looking for intersections
                    for h in self.grid:
                        interstices(self, (h[0], h[1]), (h[2], h[3]), self.paths, self.hatches, self.options.holdBackHatchFromEdges, self.options.holdBackSteps)

            elif node.tag in [inkex.addNS('ellipse', 'svg'), 'ellipse',
                              inkex.addNS('circle', 'svg'), 'circle']:

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

                if node.tag in [inkex.addNS('ellipse', 'svg'), 'ellipse']:
                    rx = float(node.get('rx', '0'))
                    ry = float(node.get('ry', '0'))
                else:
                    rx = float(node.get('r', '0'))
                    ry = rx

                cx = float(node.get('cx', '0'))
                cy = float(node.get('cy', '0'))
                x1 = cx - rx
                x2 = cx + rx

                d = 'M {x1:f},{cy:f} ' \
                    'A {rx:f},{ry:f} ' \
                    '0 1 0 {x2:f},{cy:f} ' \
                    'A {rx:f},{ry:f} ' \
                    '0 1 0 {x1:f},{cy:f}'.format(x1=x1,
                                                 x2=x2,
                                                 rx=rx,
                                                 ry=ry,
                                                 cy=cy)
                self.addPathVertices(d, node, mat_new)
                # We now have a path we want to apply a (cross)hatch to
                # Apply appropriate functions
                b_have_grid = self.makeHatchGrid(float(self.options.hatchAngle), float(self.options.hatchSpacing), True)
                if b_have_grid:
                    if self.options.crossHatch:
                        self.makeHatchGrid(float(self.options.hatchAngle + 90.0), float(self.options.hatchSpacing), False)
                    # Now loop over our hatch lines looking for intersections
                    for h in self.grid:
                        interstices(self, (h[0], h[1]), (h[2], h[3]), self.paths, self.hatches, self.options.holdBackHatchFromEdges, self.options.holdBackSteps)

            elif node.tag in [inkex.addNS('pattern', 'svg'), 'pattern']:
                pass
            elif node.tag in [inkex.addNS('metadata', 'svg'), 'metadata']:
                pass
            elif node.tag in [inkex.addNS('defs', 'svg'), 'defs']:
                pass
            elif node.tag in [inkex.addNS('namedview', 'sodipodi'), 'namedview']:
                pass
            elif node.tag in [inkex.addNS('eggbot', 'svg'), 'eggbot']:
                pass
            elif node.tag in [inkex.addNS('WCB', 'svg'), 'WCB']:
                pass
            elif node.tag in [inkex.addNS('text', 'svg'), 'text']:
                inkex.errormsg('Warning: unable to draw text, please convert it to a path first.')
                pass
            elif not isinstance(node.tag, basestring):
                pass
            else:
                inkex.errormsg('Warning: unable to hatch object <{0}>, please convert it to a path first.'.format(node.tag))
                pass

    def joinFillsWithNode(self, node, stroke_width, path):

        """
        Generate a SVG <path> element containing the path data "path".
        Then put this new <path> element into a <group> with the supplied
        node.  This means making a new <group> element and moving node
        under it with the new <path> as a sibling element.
        """

        if not path or len(path) == 0:
            return

        # Make a new SVG <group> element whose parent is the parent of node
        parent = node.getparent()
        if parent is None:
            parent = self.document.getroot()
        g = inkex.etree.SubElement(parent, inkex.addNS('g', 'svg'))
        # Move node to be a child of this new <g> element
        g.append(node)

        # Now make a <path> element which contains the hatches & is a child
        # of the new <g> element
        stroke_color = '#000000'  # default assumption
        stroke_width = '1.0'  # default value

        try:
            style = node.get('style')
            if style is not None:
                declarations = style.split(';')
                for i, declaration in enumerate(declarations):
                    parts = declaration.split(':', 2)
                    if len(parts) == 2:
                        (prop, val) = parts
                        prop = prop.strip().lower()
                        if prop == 'stroke-width':
                            stroke_width = val.strip()
                        elif prop == 'stroke':
                            val = val.strip()
                            stroke_color = val
        finally:
            style = {'stroke': '{0}'.format(stroke_color), 'fill': 'none', 'stroke-width': '{0}'.format(stroke_width)}
            line_attribs = {'style': simplestyle.formatStyle(style), 'd': path}
            tran = node.get('transform')
            if tran is not None and tran != '':
                line_attribs['transform'] = tran
            inkex.etree.SubElement(g, inkex.addNS('path', 'svg'), line_attribs)

    def makeHatchGrid(self, angle, spacing, init=True):  # returns True if succeeds in making grid, else False

        """
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
        """

        # If this is the first call, do some one time initializations
        # When generating cross hatches, we may be called more than once
        if init:
            self.getBoundingBox()
            self.grid = []

        # Determine the width and height of the bounding box containing
        # all the polygons to be hatched
        w = self.xmax - self.xmin
        h = self.ymax - self.ymin

        b_bounding_box_exists = ((w != (EXTREME_NEG - EXTREME_POS)) and (h != (EXTREME_NEG - EXTREME_POS)))
        ret_value = b_bounding_box_exists

        if b_bounding_box_exists:
            # Nice thing about rectangles is that the diameter of the circle
            # encompassing them is the length the rectangle's diagonal...
            r = math.sqrt(w * w + h * h) / 2.0

            # Length of a hatch line will be 2r
            # Now generate hatch lines within the square
            # centered at (0, 0) and with side length at least d

            # While we could generate these lines running back and forth,
            # that makes for weird behavior later when applying odd/even
            # rules AND there are nested polygons.  Instead, when we
            # generate the SVG <path> elements with the hatch line
            # segments, we can do the back and forth weaving.

            # Rotation information
            ca = math.cos(math.radians(90 - angle))
            sa = math.sin(math.radians(90 - angle))

            # Translation information
            cx = self.xmin + (w / 2)
            cy = self.ymin + (h / 2)

            # Since the spacing may be fractional (e.g., 6.5), we
            # don't try to use range() or other integer iterator
            spacing = float(abs(spacing))
            i = -r
            while i <= r:
                # Line starts at (i, -r) and goes to (i, +r)
                x1 = cx + (i * ca) + (r * sa)  # i * ca - (-r) * sa
                y1 = cy + (i * sa) - (r * ca)  # i * sa + (-r) * ca
                x2 = cx + (i * ca) - (r * sa)  # i * ca - (+r) * sa
                y2 = cy + (i * sa) + (r * ca)  # i * sa + (+r) * ca
                i += spacing
                # Remove any potential hatch lines which are entirely
                # outside of the bounding box
                if (x1 < self.xmin and x2 < self.xmin) or (x1 > self.xmax and x2 > self.xmax):
                    continue
                if (y1 < self.ymin and y2 < self.ymin) or (y1 > self.ymax and y2 > self.ymax):
                    continue
                self.grid.append((x1, y1, x2, y2))

        return ret_value

    def effect(self):

        global ref_count
        global pt_last_position_abs
        # Viewbox handling
        self.handleViewBox()

        ref_count = 0
        pt_last_position_abs = [0, 0]

        # Build a list of the vertices for the document's graphical elements
        if self.options.ids:
            # Traverse the selected objects
            for id_ in self.options.ids:
                self.recursivelyTraverseSvg([self.selected[id_]], self.docTransform)
        else:
            # Traverse the entire document
            self.recursivelyTraverseSvg(self.document.getroot(), self.docTransform)

        # After recursively traversing the svg, we will have a dictionary of transforms and hatches
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
        s = stroke_width_target / math.sqrt(2)

        # Now, dump the hatch fills sorted by which document element
        # they correspond to.  This is made easy by the fact that we
        # saved the information and used each element's lxml.etree node
        # pointer as the dictionary key under which to save the hatch
        # fills for that node.

        abs_line_segments = {} # Absolute line segments
        n_abs_line_segment_total = 0
        n_pen_lifts = 0
        # To implement
        for key in self.hatches:
            direction = True
            if key in self.transforms:
                transform = inverseTransform(self.transforms[key])
                # Determine the scaled stroke width for a hatch line
                # We produce a line segment of unit length, transform
                # its endpoints and then determine the length of the
                # resulting line segment.
                pt1 = [0, 0]
                pt2 = [s, s]
                applyTransformToPoint(transform, pt1)
                applyTransformToPoint(transform, pt2)
                dx = pt2[0] - pt1[0]
                dy = pt2[1] - pt1[1]
                stroke_width = math.sqrt(dx * dx + dy * dy)
            else:
                transform = None
                stroke_width = 1.0

            # The transform also applies to the hatch spacing we use when searching for end connections
            transformed_hatch_spacing = stroke_width * self.options.hatchSpacing

            path = ''  # regardless of whether or not we're reducing pen lifts
            pt_last_position_abs = [0, 0]
            pt_last_position_abs[0] = 0
            pt_last_position_abs[1] = 0
            f_distance_moved_with_pen_up = 0
            if not self.options.reducePenLifts:
                for segment in self.hatches[key]:
                    if len(segment) < 2:
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
                    # after the fact (i.e., what's this transform here for?).
                    # So, we compute the inverse transform and apply it here.
                    if transform is not None:
                        applyTransformToPoint(transform, pt1)
                        applyTransformToPoint(transform, pt2)
                    # Now generate the path data for the <path>
                    if direction:
                        # Go this direction
                        path += ('M {0:f},{1:f} l {2:f},{3:f} '.format(pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]))
                    else:
                        # Or go this direction
                        path += ('M {0:f},{1:f} l {2:f},{3:f} '.format(pt2[0], pt2[1], pt1[0] - pt2[0], pt1[1] - pt2[1]))

                    direction = not direction
                self.joinFillsWithNode(key, stroke_width, path[:-1])

            else:
                for segment in self.hatches[key]:
                    if len(segment) < 2:  # Copied from original, no idea why this is needed [sbm]
                        continue
                    if direction:
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
                    # after the fact (i.e., what's this transform here for?).
                    # So, we compute the inverse transform and apply it here.
                    if transform is not None:
                        applyTransformToPoint(transform, pt1)
                        applyTransformToPoint(transform, pt2)

                    # Now generate the path data for the <path>
                    # BUT we want to combine as many paths as possible to reduce pen lifts.
                    # In order to combine paths, we need to know all of the path segments.
                    # The solution to this conundrum is to generate all path segments,
                    # but instead of drawing them into the path right away, we put them in
                    # an array where they'll be available for random access
                    # by our anti-pen-lift algorithm
                    abs_line_segments[n_abs_line_segment_total] = [pt1, pt2, False]  # False indicates that segment has not yet been drawn
                    n_abs_line_segment_total += 1
                    direction = not direction

                # Now have a nice juicy buffer full of line segments with absolute coordinates
                f_proposed_neighborhood_radius_squared = self.ProposeNeighborhoodRadiusSquared(transformed_hatch_spacing)  
                # Just fixed and simple for now - may make function of neighborhood later
                
                for ref_count in range(n_abs_line_segment_total):  # This is the entire range of segments,
                    # Sets global ref_count to segment which has an end closest to current pen position.
                    # Doesn't need to select which end is closest, as that will happen below, with n_ref_end_index.
                    # When we have gone thru this whole range, we will be completely done.
                    # We only get here again, after all _connected_ segments have been "drawn".
                    if not abs_line_segments[ref_count][2]:  # Test whether this segment has been drawn
                        # Has not been drawn yet

                        # Before we do any irrevocable changes to path, let's see if we are going to be able to append any segments.
                        # The below solution is inelegant, but has the virtue of being relatively simple to implement.
                        # Pre-qualify this segment on the issue of whether it has any connecting segments.
                        # If it does not, then just add the path for this one segment, and go on to the next.
                        # If it does have connecting segments, we need to go through the recursive logic.
                        # Lazily, again, select the desired direction of line ahead of time.

                        b_found_segment_to_add = False  # default assumption
                        n_ref_end_index_at_closest = 0
                        f_closest_distance_squared = 123456  # just a random large number
                        for n_ref_end_index in range(2):
                            pt_reference = abs_line_segments[ref_count][n_ref_end_index]
                            pt_reference_other_end = abs_line_segments[ref_count][not n_ref_end_index]
                            f_reference_direction_radians = math.atan2(pt_reference_other_end[1] - pt_reference[1], pt_reference_other_end[0] - pt_reference[0])  # from other end to this end
                            # The following is just a simple copy from the routine in recursivelyAppendNearbySegments procedure
                            # Look through all possibilities to choose the closest that fulfills all requirements e.g. direction and colinearity
                            for innerCount in range(n_abs_line_segment_total):  # investigate all segments
                                if not abs_line_segments[innerCount][2]:
                                    # This segment currently undrawn, so it is a candidate for a path extension
                                    # Need to check both ends of each and every proposed segment so we can find the most appropriate one
                                    # Define pt2 in the reference as the end which we want to extend
                                    for nNewSegmentInitialEndIndex in range(2):
                                        # First try initial end of test segment (aka pt1) vs final end (aka pt2) of reference segment
                                        if innerCount != ref_count:  # don't investigate self ends
                                            delta_x = abs_line_segments[innerCount][nNewSegmentInitialEndIndex][0] - pt_reference[0]  # proposed initial pt1 X minus existing final pt1 X
                                            delta_y = abs_line_segments[innerCount][nNewSegmentInitialEndIndex][1] - pt_reference[1]  # proposed initial pt1 Y minus existing final pt1 Y
                                            if (delta_x * delta_x + delta_y * delta_y) < f_proposed_neighborhood_radius_squared:
                                                f_this_distance_squared = delta_x * delta_x + delta_y * delta_y
                                                pt_new_segment_this_end = abs_line_segments[innerCount][nNewSegmentInitialEndIndex]
                                                pt_new_segment_other_end = abs_line_segments[innerCount][not nNewSegmentInitialEndIndex]
                                                f_new_segment_direction_radians = math.atan2(pt_new_segment_this_end[1] - pt_new_segment_other_end[1], pt_new_segment_this_end[0] - pt_new_segment_other_end[0])  # from other end to this end
                                                # If this end would cause an alternating direction,
                                                # then exclude it
                                                if not self.WouldBeAnAlternatingDirection(f_reference_direction_radians, f_new_segment_direction_radians):
                                                    pass
                                                elif f_this_distance_squared < f_closest_distance_squared:
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
                                                    f_joiner_direction_radians = math.atan2(pt_new_segment_this_end[1] - pt_reference[1], pt_new_segment_this_end[0] - pt_reference[0])
                                                    if not self.AreCoLinear(f_reference_direction_radians, f_joiner_direction_radians):
                                                        # not colinear
                                                        f_closest_distance_squared = f_this_distance_squared
                                                        b_found_segment_to_add = True
                                                        n_ref_end_index_at_closest = n_ref_end_index

                        # At last we've looked at all the candidate segment ends, as related to all the reference ends
                        if not b_found_segment_to_add:
                            # This segment is solitary.
                            # Must start a new line, not joined to any previous paths
                            delta_x = abs_line_segments[ref_count][1][0] - abs_line_segments[ref_count][0][0]  # end minus start, in original direction
                            delta_y = abs_line_segments[ref_count][1][1] - abs_line_segments[ref_count][0][1]  # end minus start, in original direction
                            path += ('M {0:f},{1:f} l {2:f},{3:f} '.format(abs_line_segments[ref_count][0][0],
                                                                           abs_line_segments[ref_count][0][1],
                                                                           delta_x,
                                                                           delta_y))  # delta is from initial point
                            f_distance_moved_with_pen_up += math.hypot(
                                    abs_line_segments[ref_count][0][0] - pt_last_position_abs[0],
                                    abs_line_segments[ref_count][0][1] - pt_last_position_abs[1])
                            pt_last_position_abs[0] = abs_line_segments[ref_count][0][0] + delta_x
                            pt_last_position_abs[1] = abs_line_segments[ref_count][0][1] + delta_y
                            abs_line_segments[ref_count][2] = True  # True flags that this line segment has been
                            # added to the path to be drawn, so should
                            # no longer be a candidate for any kind of move.
                            n_pen_lifts += 1
                        else:
                            # Found segment to add, and we must get to it in absolute terms
                            delta_x = (abs_line_segments[ref_count][n_ref_end_index_at_closest][0] -
                                       abs_line_segments[ref_count][not n_ref_end_index_at_closest][0])
                            # final point (which was closer to the closest continuation segment) minus initial point = delta_x

                            delta_y = (abs_line_segments[ref_count][n_ref_end_index_at_closest][1] -
                                       abs_line_segments[ref_count][not n_ref_end_index_at_closest][1])
                            # final point (which was closer to the closest continuation segment) minus initial point = delta_y

                            path += ('M {0:f},{1:f} l '.format(abs_line_segments[ref_count][not n_ref_end_index_at_closest][0],
                                                               abs_line_segments[ref_count][not n_ref_end_index_at_closest][1]))
                            f_distance_moved_with_pen_up += math.hypot(
                                    abs_line_segments[ref_count][not n_ref_end_index_at_closest][0] - pt_last_position_abs[0],
                                    abs_line_segments[ref_count][not n_ref_end_index_at_closest][1] - pt_last_position_abs[1])
                            pt_last_position_abs[0] = abs_line_segments[ref_count][not n_ref_end_index_at_closest][0]
                            pt_last_position_abs[1] = abs_line_segments[ref_count][not n_ref_end_index_at_closest][1]
                            # Note that this does not complete the line, as the completion (the delta_x, delta_y part) is being held in abeyance

                            # We are coming up on a problem:
                            # If we add a curve to the end of the line, we have made the curve extend beyond the end of the line,
                            # and thus beyond the boundaries we should be respecting.
                            # The solution is to hold in abeyance the actual plotting of the line,
                            # holding it available for shrinking if a curve is to be added.
                            # That is
                            relative_held_line_pos = {0: delta_x, 1: delta_y}
                            # delta is from initial point
                            # Will be printed after we know if it must be modified
                            # to keep the ending join within bounds
                            pt_last_position_abs[0] += delta_x
                            pt_last_position_abs[1] += delta_y

                            abs_line_segments[ref_count][2] = True  # True flags that this line segment has been
                            # added to the path to be drawn, so should
                            # no longer be a candidate for any kind of move.
                            n_pen_lifts += 1
                            # Now comes the speedup logic:
                            # We've just drawn a segment starting at an absolute, not relative, position.
                            # It was drawn from pt1 to pt2.
                            # Look for an as-yet-not-drawn segment which has a beginning or ending
                            # point "near" the end point of this absolute draw, and leave the pen down
                            # while moving to and then drawing this found line.
                            # Do this recursively, marking each segment True to show that
                            # it has been "drawn" already.
                            # pt2 is the reference point, ie. the point from which the next segment will start
                            path = self.recursivelyAppendNearbySegments(transformed_hatch_spacing,
                                                                            0,
                                                                            ref_count,
                                                                            n_ref_end_index_at_closest,
                                                                            n_abs_line_segment_total,
                                                                            abs_line_segments,
                                                                            path,
                                                                            relative_held_line_pos)

                self.joinFillsWithNode(key, stroke_width, path[:-1])

    def recursivelyAppendNearbySegments(self,
                                            transformed_hatch_spacing,
                                            n_recursion_count,
                                            n_ref_segment_count,
                                            n_ref_end_index,
                                            n_abs_line_segment_total,
                                            abs_line_segments,
                                            cumulative_path,
                                            relative_held_line_pos):

        global pt_last_position_abs
        f_proposed_neighborhood_radius_squared = self.ProposeNeighborhoodRadiusSquared(transformed_hatch_spacing)

        # Look through all possibilities to choose the closest
        b_found_segment_to_add = False  # default assumption
        n_new_segment_end1_index_at_closest = 0
        n_outer_count_at_closest = -1
        f_closest_distance_squared = 123456789.0  # just a random large number

        pt_reference = abs_line_segments[n_ref_segment_count][n_ref_end_index]
        pt_reference_other_end = abs_line_segments[n_ref_segment_count][not n_ref_end_index]
        f_reference_delta_x = pt_reference_other_end[0] - pt_reference[0]
        f_reference_delta_y = pt_reference_other_end[1] - pt_reference[1]
        f_reference_direction_radians = math.atan2(f_reference_delta_y, f_reference_delta_x)  # from other end to this end

        for outerCount in range(n_abs_line_segment_total):  # investigate all segments
            if not abs_line_segments[outerCount][2]:
                # This segment currently undrawn, so it is a candidate for a path extension

                # Need to check both ends of each and every proposed segment until we find one in the neighborhood
                # Defines pt2 in the reference as the end which we want to extend

                for n_new_segment_end1_index in range(2):
                    # First try initial end of test segment (aka pt1) vs final end (aka pt2) of reference segment
                    if outerCount != n_ref_segment_count:  # don't investigate self ends
                        delta_x = abs_line_segments[outerCount][n_new_segment_end1_index][0] - pt_reference[0]  # proposed initial pt1 X minus existing final pt1 X
                        delta_y = abs_line_segments[outerCount][n_new_segment_end1_index][1] - pt_reference[1]  # proposed initial pt1 Y minus existing final pt1 Y
                        if (delta_x * delta_x + delta_y * delta_y) < f_proposed_neighborhood_radius_squared:
                            f_this_distance_squared = delta_x * delta_x + delta_y * delta_y
                            pt_new_segment_this_end = abs_line_segments[outerCount][n_new_segment_end1_index]
                            pt_new_segment_other_end = abs_line_segments[outerCount][not n_new_segment_end1_index]
                            f_new_segment_Dx = pt_new_segment_this_end[0] - pt_new_segment_other_end[0]
                            f_new_segment_Dy = pt_new_segment_this_end[1] - pt_new_segment_other_end[1]
                            f_new_segment_direction_radians = math.atan2(f_new_segment_Dy, f_new_segment_Dx)  # from other end to this end
                            if not self.WouldBeAnAlternatingDirection(f_reference_direction_radians, f_new_segment_direction_radians):
                                # If this end would cause an alternating direction,
                                # then exclude it regardless of how close it is
                                pass

                            elif f_this_distance_squared < f_closest_distance_squared:
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

                                f_joiner_direction_radians = math.atan2(pt_new_segment_this_end[1] - pt_reference[1], pt_new_segment_this_end[0] - pt_reference[0])
                                if not self.AreCoLinear(f_reference_direction_radians, f_joiner_direction_radians):
                                    # not colinear
                                    f_closest_distance_squared = f_this_distance_squared
                                    b_found_segment_to_add = True
                                    n_new_segment_end1_index_at_closest = n_new_segment_end1_index
                                    n_outer_count_at_closest = outerCount
                                    delta_x_at_closest = delta_x
                                    delta_y_at_closest = delta_y

        # At last we've looked at all the candidate segment ends
        n_recursion_count += 1
        if not b_found_segment_to_add or n_recursion_count >= RECURSION_LIMIT:
            cumulative_path += '{0:f},{1:f} '.format(relative_held_line_pos[0],
                                                     relative_held_line_pos[1])  # close out this segment
            pt_last_position_abs[0] += relative_held_line_pos[0]
            pt_last_position_abs[1] += relative_held_line_pos[1]
            return cumulative_path  # No undrawn segments were suitable for appending,
            # or there were so many that we worry about python recursion limit
        else:
            n_new_segment_end1_index = n_new_segment_end1_index_at_closest
            n_new_segment_end2_index = not n_new_segment_end1_index
            # n_new_segment_end1_index is 0 for connecting to pt1,
            # and is 1 for connecting to pt2
            count = n_outer_count_at_closest  # count is the index of the segment to be appended.
            delta_x = delta_x_at_closest  # delta from final end of incoming segment to initial end of outgoing segment
            delta_y = delta_y_at_closest

            # First, move pen to initial end (may be either its pt1 or its pt2) of new segment

            # Insert a bezier curve for this transition element
            # To accomplish this, we need information on the incoming and outgoing segments.
            # Specifically, we need to know the lengths and angles of the segments in
            # order to decide on control points.
            f_in_Dx = abs_line_segments[n_ref_segment_count][n_ref_end_index][0] - abs_line_segments[n_ref_segment_count][not n_ref_end_index][0]
            f_in_Dy = abs_line_segments[n_ref_segment_count][n_ref_end_index][1] - abs_line_segments[n_ref_segment_count][not n_ref_end_index][1]
            # The outgoing deltas are based on the reverse direction of the segment, i.e. the segment pointing back to the joiner bezier curve
            f_out_Dx = abs_line_segments[count][n_new_segment_end1_index][0] - abs_line_segments[count][n_new_segment_end2_index][0]  # index is [count][start point = 0, final point = 1][0=x, 1=y]
            f_out_Dy = abs_line_segments[count][n_new_segment_end1_index][1] - abs_line_segments[count][n_new_segment_end2_index][1]

            length_of_incoming = math.hypot(f_in_Dx, f_in_Dy)
            length_of_outgoing = math.hypot(f_out_Dx, f_out_Dy)

            # We are going to trim-up the ends of the incoming and outgoing segments,
            # in order to get a curve which reliably does not extend beyond the boundary.
            # Crude readings from inkscape on bezier curve overshoot, using control points extended hatch-spacing distance parallel to segment:
            # when end points are in line, overshoot 12/16 in direction of segment
            #          when at 45 degrees, overshoot 12/16 in direction of segment
            #          when at 60 degrees, overshoot 12/16 in direction of segment
            # Conclusion, at any angle, remove 0.75 * hatch spacing from the length of both lines,
            # where 0.75 is, by no coincidence, BEZIER_OVERSHOOT_MULTIPLIER

            # If hatches are getting quite short, we can use a smaller Bezier loop at
            # the end to squeeze into smaller spaces.  We'll use a normal nice smooth
            # curve for non-short hatches
            f_desired_shorten_for_smoothest_join = transformed_hatch_spacing * BEZIER_OVERSHOOT_MULTIPLIER  # This is what we really want to use for smooth curves
            # Separately check incoming vs outgoing lengths to see if bezier distances must be reduced,
            # then choose greatest reduction to apply to both - lest we go off-course
            # Finally, clip reduction to be no less than 1.0
            f_control_point_divider_incoming = 2.0 * f_desired_shorten_for_smoothest_join / length_of_incoming
            f_control_point_divider_outgoing = 2.0 * f_desired_shorten_for_smoothest_join / length_of_outgoing
            if f_control_point_divider_incoming > f_control_point_divider_outgoing:
                f_largest_desired_control_point_divider = f_control_point_divider_incoming
            else:
                f_largest_desired_control_point_divider = f_control_point_divider_outgoing
            if f_largest_desired_control_point_divider < 1.0:
                f_control_point_divider = 1.0
            else:
                f_control_point_divider = f_largest_desired_control_point_divider
            f_desired_shorten = f_desired_shorten_for_smoothest_join / f_control_point_divider

            pt_delta_to_subtract_from_incoming_end = self.RelativeControlPointPosition(f_desired_shorten, f_in_Dx, f_in_Dy, 0, 0)
            # Note that this will be subtracted from the _point held in abeyance_.
            relative_held_line_pos[0] -= pt_delta_to_subtract_from_incoming_end[0]
            relative_held_line_pos[1] -= pt_delta_to_subtract_from_incoming_end[1]

            pt_delta_to_add_to_outgoing_start = self.RelativeControlPointPosition(f_desired_shorten, f_out_Dx, f_out_Dy, 0, 0)

            # We know that when we tack on a curve, we must chop some off the end of the incoming segment,
            # and also chop some off the start of the outgoing segment.
            # Now, we know we want the control points to be on a projection of each segment,
            # in order that there be no abrupt change of plotting angle.  The question is, how
            # far beyond the endpoint should we place the control point.
            pt_relative_control_point_in = self.RelativeControlPointPosition(
                    transformed_hatch_spacing / f_control_point_divider,
                    f_in_Dx,
                    f_in_Dy,
                    0,
                    0)
            pt_relative_control_point_out = self.RelativeControlPointPosition(
                    transformed_hatch_spacing / f_control_point_divider,
                    f_out_Dx,
                    f_out_Dy,
                    delta_x,
                    delta_y)

            cumulative_path += '{0:f},{1:f} '.format(relative_held_line_pos[0],
                                                     relative_held_line_pos[1])  # close out this segment, which has been modified
            pt_last_position_abs[0] += relative_held_line_pos[0]
            pt_last_position_abs[1] += relative_held_line_pos[1]
            # add bezier cubic curve
            cumulative_path += ('c {0:f},{1:f} {2:f},{3:f} {4:f},{5:f} l '.format(pt_relative_control_point_in[0],
                                                                                  pt_relative_control_point_in[1],
                                                                                  pt_relative_control_point_out[0],
                                                                                  pt_relative_control_point_out[1],
                                                                                  delta_x,
                                                                                  delta_y))
            pt_last_position_abs[0] += delta_x
            pt_last_position_abs[1] += delta_y
            # Next, move pen in appropriate direction to draw the new segment, given that
            # we have just moved to the initial end of the new segment.
            # This needs special treatment, as we just did some length changing.
            delta_x = abs_line_segments[count][n_new_segment_end2_index][0] - abs_line_segments[count][n_new_segment_end1_index][0] + pt_delta_to_add_to_outgoing_start[0]
            delta_y = abs_line_segments[count][n_new_segment_end2_index][1] - abs_line_segments[count][n_new_segment_end1_index][1] + pt_delta_to_add_to_outgoing_start[1]
            relative_held_line_pos[0] = delta_x  # delta is from initial point
            relative_held_line_pos[1] = delta_y  # Will be printed after we know if it must be modified

            # Mark this segment as drawn
            abs_line_segments[count][2] = True

            cumulative_path = self.recursivelyAppendNearbySegments(transformed_hatch_spacing,
                                                                       n_recursion_count,
                                                                       count,
                                                                       n_new_segment_end2_index,
                                                                       n_abs_line_segment_total,
                                                                       abs_line_segments,
                                                                       cumulative_path,
                                                                       relative_held_line_pos)
            return cumulative_path

    def ProposeNeighborhoodRadiusSquared(self, transformed_hatch_spacing):
        return transformed_hatch_spacing * transformed_hatch_spacing * self.options.hatchScope * self.options.hatchScope
        # The multiplier of x generates a radius of x^0.5 times the hatch spacing.

    @staticmethod
    def RelativeControlPointPosition(distance, f_delta_x, f_delta_y, delta_x, delta_y):

        # returns the point, relative to 0, 0 offset by delta_x, delta_y,
        # which extends a distance of "distance" at a slope defined by f_delta_x and f_delta_y
        pt_return = [0, 0]

        if f_delta_x == 0:
            pt_return[0] = delta_x
            pt_return[1] = math.copysign(distance, f_delta_y) + delta_y
        elif f_delta_y == 0:
            pt_return[0] = math.copysign(distance, f_delta_x) + delta_x
            pt_return[1] = delta_y
        else:
            f_slope = math.atan2(f_delta_y, f_delta_x)
            pt_return[0] = distance * math.cos(f_slope) + delta_x
            pt_return[1] = distance * math.sin(f_slope) + delta_y

        return pt_return

    @staticmethod
    def WouldBeAnAlternatingDirection(f_reference_direction_radians, f_new_segment_direction_radians):
        # atan2 returns values in the range -pi to +pi, so we must evaluate difference values
        # in the range of -2*pi to +2*pi
        # f_dir_diff_rad:  Direction difference, radians
        f_dir_diff_rad = f_reference_direction_radians - f_new_segment_direction_radians
        if f_dir_diff_rad < 0:
            f_dir_diff_rad += 2 * math.pi
        # Without having changed the vector direction of the difference, we have
        # now reduced the range to 0 to 2*pi
        f_dir_diff_rad -= math.pi  # flip opposite direction to coincide with same direction
        # Of course they may not be _exactly_ pi different due to osmosis, so allow a tolerance
        b_ret_val = abs(f_dir_diff_rad) < RADIAN_TOLERANCE_FOR_ALTERNATING_DIRECTION

        return b_ret_val

    @staticmethod
    def AreCoLinear(f_direction_1_radians, f_direction_2_radians):
        # allow slight difference in angles, for floating-point indeterminacy
        f_abs_delta_radians = abs(f_direction_1_radians - f_direction_2_radians)
        if f_abs_delta_radians < RADIAN_TOLERANCE_FOR_COLINEAR:
            return True
        elif abs(f_abs_delta_radians - math.pi) < RADIAN_TOLERANCE_FOR_COLINEAR:
            return True
        else:
            return False


if __name__ == '__main__':

    e = Eggbot_Hatch()
    e.affect()
