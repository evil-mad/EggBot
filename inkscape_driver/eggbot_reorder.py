# coding=utf-8
# EggBot Path Ordering extension
# This extension tries to re-order the document's paths to improve
# the plotting time by plotting nearby paths consecutively.
#
# Written by Matthew Beckler for the EggBot project.
# Email questions and comments to matthew at mbeckler dot org
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

import gettext
import math
import random
import sys

import inkex
import simplepath
import simpletransform


def dist(x0, y0, x1, y1):
    return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def find_ordering_naive(objlist):
    """
    Takes a list of (id, (startX, startY, endX, endY)), and finds the best ordering.
    Doesn't handle anything fancy, like reversing the ordering, but it's useful for now.
    Returns a list of JUST THE IDs, in a better order, as well as the original and optimized
    "air distance" which is just the distance traveled in the air. Perhaps we want to make
    these comparison distances into something more relevant such as degrees traveled?
    """

    # let's figure out the default in-air length, so we know how much we improved
    air_length_default = 0
    try:
        oldx = objlist[0][1][2]
        oldy = objlist[0][1][3]
    except:
        inkex.errormsg(gettext.gettext(str(objlist[0])))
        sys.exit(1)
    for _, coords in objlist[1:]:
        air_length_default += dist(oldx, oldy, coords[0], coords[1])
        oldx = coords[2]
        oldy = coords[3]

    air_length_ordered = 0
    # for now, start with a random one:
    sort_list = []
    random_index = random.randint(0, len(objlist) - 1)
    sort_list.append(objlist[random_index])
    objlist.remove(objlist[random_index])

    # for now, do this in the most naive way:
    # for the previous end point, iterate over each remaining path and pick the closest starting point
    while objlist:
        min_distance = float('inf') # Set the inital min distance to infinity.
        for path in objlist:
            # instead of having a prevX, prevY, we just look at the last item in sort_list
            this_distance = dist(sort_list[-1][1][2], sort_list[-1][1][3], path[1][0], path[1][1])
            # this is such a common thing to do, you'd think there would be a name for it...
            if this_distance < min_distance:
                min_distance = this_distance
                min_path = path
        air_length_ordered += min_distance
        sort_list.append(min_path)
        objlist.remove(min_path)

    # Explicitly cast to float for use later.
    air_length_default = float(air_length_default)
    air_length_ordered = float(air_length_ordered)

    # remove the extraneous info from the list order
    sort_order = [id for id, coords in sort_list]
    return sort_order, air_length_default, air_length_ordered


def conv(x, y, trans_matrix=None):
    """
    not used currently, but can be used to apply a translation matrix to an (x, y) pair
    I'm sure there is a better way to do this using simpletransform or it's ilk
    """

    if trans_matrix:
        xt = trans_matrix[0][0] * x + trans_matrix[0][1] * y + trans_matrix[0][2]
        yt = trans_matrix[1][0] * x + trans_matrix[1][1] * y + trans_matrix[1][2]
        return xt, yt
    else:
        return x, y


class EggBotReorderPaths(inkex.Effect):
    def __init__(self):
        inkex.Effect.__init__(self)

    def get_start_end(self, node, transform):
        """Given a node, return the start and end points"""
        d = node.get('d')
        sp = simplepath.parsePath(d)

        # simplepath converts coordinates to absolute and cleans them up, but
        # these are still some big assumptions here, are they always valid? TODO
        start_x = sp[0][1][0]
        start_y = sp[0][1][1]
        if sp[-1][0] == 'Z':
            # go back to start
            end_x = start_x
            end_y = start_y
        else:
            end_x = sp[-1][1][-2]
            end_y = sp[-1][1][-1]

        sx, sy = conv(start_x, start_y, transform)
        ex, ey = conv(end_x, end_y, transform)
        return sx, sy, ex, ey

    def effect(self):
        """This is the main entry point"""

        # based partially on the restack.py extension
        if self.selected:

            # TODO check for non-path elements?
            # TODO it seems like the order of selection is not consistent

            # for each selected item - TODO make this be all objects, everywhere
            # I can think of two options:
            # 1. Iterate over all paths in root, then iterate over all layers, and their paths
            # 2. Some magic with xpath? (would this limit us to specific node types?)

            objlist = []
            for id_, node in self.selected.iteritems():
                transform = node.get('transform')
                if transform:
                    transform = simpletransform.parseTransform(transform)

                item = (id_, self.get_start_end(node, transform))
                objlist.append(item)

            # sort / order the objects
            sort_order, air_distance_default, air_distance_ordered = find_ordering_naive(objlist)

            for id_ in sort_order:
                # There's some good magic here, that you can use an
                # object id to index into self.selected. Brilliant!
                self.current_layer.append(self.selected[id_])

            if air_distance_default > 0:  # don't divide by zero. :P
                improvement_pct = 100 * ((air_distance_default - air_distance_ordered) / air_distance_default)
                reorder_msg = gettext.gettext(
                    "Selected paths have been reordered and optimized for quicker EggBot plotting.\n\n"
                    "Original air-distance: {0:.2f}\n"
                    "Optimized air-distance: {1:.2f}\n"
                    "Distance reduced by: {2:1.2f}%\n\n"
                    "Have a nice day!".format(
                        air_distance_default,
                        air_distance_ordered,
                        improvement_pct
                    )
                )
                inkex.errormsg(reorder_msg)
            else:
                inkex.errormsg(gettext.gettext("Unable to start. Please select multiple distinct paths. :)"))


e = EggBotReorderPaths()
e.affect()
