#!/usr/bin/env python
# coding=utf-8

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

import bezmisc
import cspsubdiv
import cubicsuperpath
import inkex
import simplepath
from simpletransform import applyTransformToPath, applyTransformToPoint, composeTransform, parseTransform

N_PAGE_WIDTH = 3200
N_PAGE_HEIGHT = 800


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


def parseLengthWithUnits(a_str):
    """
    Parse an SVG value which may or may not have units attached
    This version is greatly simplified in that it only allows: no units,
    units of px, and units of %.  Everything else, it returns None for.
    There is a more general routine to consider in scour.py if more
    generality is ever needed.
    """

    u = 'px'
    s = a_str.strip()
    if s[-2:] == 'px':
        s = s[:-2]
    elif s[-1:] == '%':
        u = '%'
        s = s[:-1]

    try:
        v = float(s)
    except:
        return None, None

    return v, u


def subdivideCubicPath(sp, flat, i=1):
    """
    [ Lifted from eggbot.py with impunity ]

    Break up a bezier curve into smaller curves, each of which
    is approximately a straight line within a given tolerance
    (the "smoothness" defined by [flat]).

    This is a modified version of cspsubdiv.cspsubdiv(): rewritten
    because recursion-depth errors on complicated line segments
    could occur with cspsubdiv.cspsubdiv().
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


class Map(inkex.Effect):

    def __init__(self):

        inkex.Effect.__init__(self)

        self.OptionParser.add_option('--smoothness', dest='smoothness',
                                     type='float', default=float(0.2), action='store',
                                     help='Curve smoothing (less for more)')

        self.OptionParser.add_option('--maxDy', dest='maxDy',
                                     type='float', default=float(5.0), action='store',
                                     help='Vertical smoothing (less for more)')

        self.cx = float(N_PAGE_WIDTH) / 2.0
        self.cy = float(N_PAGE_HEIGHT) / 2.0
        self.xmin, self.xmax = (1.0E70, -1.0E70)
        self.maxDy = float(5)
        self.paths = {}
        self.transforms = {}

        # For handling an SVG viewbox attribute, we will need to know the
        # values of the document's <svg> width and height attributes as well
        # as establishing a transform from the viewbox to the display.

        self.docWidth = float(N_PAGE_WIDTH)
        self.docHeight = float(N_PAGE_HEIGHT)
        self.docTransform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]

    def getLength(self, name, default):

        """
        Get the <svg> attribute with name "name" and default value "default"
        Parse the attribute into a value and associated units.  Then, accept
        no units (''), units of pixels ('px'), and units of percentage ('%').
        """

        s = self.document.getroot().get(name)
        if s:
            v, u = parseLengthWithUnits(s)
            if not v:
                # Couldn't parse the value
                return None
            elif (u == '') or (u == 'px'):
                return v
            elif u == '%':
                return float(default) * v / 100.0
            else:
                # Unsupported units
                return None
        else:
            # No width specified; assume the default value
            return float(default)

    def getDocProps(self):

        """
        Get the document's height and width attributes from the <svg> tag.
        Use a default value in case the property is not present or is
        expressed in units of percentages.
        """

        self.docHeight = self.getLength('height', N_PAGE_HEIGHT)
        self.docWidth = self.getLength('width', N_PAGE_WIDTH)
        if (self.docHeight is None) or (self.docWidth is None):
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
                if (vinfo[2] != 0) and (vinfo[3] != 0):
                    sx = self.docWidth / float(vinfo[2])
                    sy = self.docHeight / float(vinfo[3])
                    self.docTransform = parseTransform('scale({0:f},{1:f})'.format(sx, sy))

    def getPathVertices(self, path, node=None, transform=None, find_bbox=False):

        """
        Decompose the path data from an SVG element into individual
        subpaths, each subpath consisting of absolute move to and line
        to coordinates.  Place these coordinates into a list of polygon
        vertices.
        """

        if (not path) or (len(path) == 0):
            # Nothing to do
            return None

        # parsePath() may raise an exception.  This is okay
        sp = simplepath.parsePath(path)
        if (not sp) or (len(sp) == 0):
            # Path must have been devoid of any real content
            return None

        # Get a cubic super path
        p = cubicsuperpath.CubicSuperPath(sp)
        if (not p) or (len(p) == 0):
            # Probably never happens, but...
            return None

        if transform:
            applyTransformToPath(transform, p)

        # Now traverse the cubic super path
        subpath_list = []
        subpath_vertices = []
        for sp in p:
            if len(subpath_vertices):
                subpath_list.append(subpath_vertices)
            subpath_vertices = []
            last_csp = None
            subdivideCubicPath(sp, float(self.options.smoothness))
            for csp in sp:
                if (last_csp is not None) and (math.fabs(csp[1][1] - last_csp[1]) > self.options.maxDy):
                    dy = (csp[1][1] - last_csp[1])
                    dx = (csp[1][0] - last_csp[0])
                    nsteps = math.ceil(math.fabs(dy / self.options.maxDy))
                    for n in range(1, int(1 + nsteps)):
                        s = n / nsteps
                        subpath_vertices.append([last_csp[0] + s * dx, last_csp[1] + s * dy])
                else:
                    # Add this vertex to the list of vertices
                    subpath_vertices.append(csp[1])
                last_csp = csp[1]
                if find_bbox:
                    if last_csp[0] < self.xmin:
                        self.xmin = last_csp[0]
                    elif last_csp[0] > self.xmax:
                        self.xmax = last_csp[0]

        # Handle final subpath
        if len(subpath_vertices):
            subpath_list.append(subpath_vertices)

        if len(subpath_list) > 0:
            self.paths[node] = subpath_list
            self.transforms[node] = transform

    def mapPathVertices(self, node):

        steps2rads = math.pi / float(1600)

        transform = self.transforms[node]
        if transform is None:
            inv_transform = None
        else:
            inv_transform = inverseTransform(transform)

        new_path = ''
        for subpath in self.paths[node]:
            last_point = subpath[0]
            last_point[0] = self.cx + (last_point[0] - self.cx) / math.cos((last_point[1] - self.cy) * steps2rads)
            if inv_transform is not None:
                applyTransformToPoint(inv_transform, last_point)
            new_path += ' M {0:f},{1:f}'.format(last_point[0], last_point[1])
            for point in subpath[1:]:
                x = self.cx + (point[0] - self.cx) / math.cos((point[1] - self.cy) * steps2rads)
                pt = [x, point[1]]
                if inv_transform is not None:
                    applyTransformToPoint(inv_transform, pt)
                new_path += ' l {0:f},{1:f}'.format(pt[0] - last_point[0], pt[1] - last_point[1])
                last_point = pt

        self.paths[node] = new_path

    def recursivelyTraverseSvg(self, a_node_list, mat_current=None, parent_visibility='visible', find_bbox=False):

        """
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
        """

        if mat_current is None:
            mat_current = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]

        for node in a_node_list:

            # Ignore invisible nodes
            v = node.get('visibility', parent_visibility)
            if v == 'inherit':
                v = parent_visibility
            if v == 'hidden' or v == 'collapse':
                pass

            # First apply the current matrix transform to this node's transform
            mat_new = composeTransform(mat_current, parseTransform(node.get("transform")))

            if node.tag in [inkex.addNS('g', 'svg'), 'g']:
                self.recursivelyTraverseSvg(node, mat_new, v, find_bbox)

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
                    if (x != 0) or (y != 0):
                        mat_new2 = composeTransform(mat_new, parseTransform('translate({0:f},{1:f})'.format(x, y)))
                    else:
                        mat_new2 = mat_new
                    v = node.get('visibility', v)
                    self.recursivelyTraverseSvg(refnode, mat_new2, v, find_bbox)

            elif node.tag == inkex.addNS('path', 'svg'):
                path_data = node.get('d')
                if path_data:
                    self.getPathVertices(path_data, node, mat_new, find_bbox)

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
                self.getPathVertices(simplepath.formatPath(a), node, mat_new, find_bbox)

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
                self.getPathVertices(simplepath.formatPath(a), node, mat_new, find_bbox)

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

                pa = pl.split()
                d = "".join(["M " + pa[i] if i == 0 else " L " + pa[i] for i in range(0, len(pa))])
                self.getPathVertices(d, node, mat_new, find_bbox)

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
                d = "".join(["M " + pa[i] if i == 0 else " L " + pa[i] for i in range(len(pa))])
                d += " Z"
                self.getPathVertices(d, node, mat_new, find_bbox)

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

                self.getPathVertices(d, node, mat_new, find_bbox)

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

            elif node.tag in [inkex.addNS('text', 'svg'), 'text']:
                inkex.errormsg('Warning: unable to draw text, please convert it to a path first.')
                pass

            elif not isinstance(node.tag, basestring):
                pass

            else:
                inkex.errormsg('Warning: unable to draw object <{0}>, please convert it to a path first.'.format(node.tag))
                pass

    def recursivelyReplaceSvg(self, nodes, parent_visibility='visible'):

        for i in range(len(nodes)):

            node = nodes[i]

            # Ignore invisible nodes
            v = node.get('visibility', parent_visibility)
            if v == 'inherit':
                v = parent_visibility
            if v == 'hidden' or v == 'collapse':
                pass

            if node.tag in [inkex.addNS('g', 'svg'), 'g']:

                self.recursivelyReplaceSvg(node, parent_visibility=v)

            elif node.tag == inkex.addNS('path', 'svg'):

                if node in self.paths:
                    # Change the path data to be the new path
                    node.set('d', self.paths[node][1:])
                    del self.paths[node]

            elif node.tag in [inkex.addNS('use', 'svg'), 'use',
                              inkex.addNS('rect', 'svg'), 'rect',
                              inkex.addNS('line', 'svg'), 'line',
                              inkex.addNS('polyline', 'svg'), 'polyline',
                              inkex.addNS('polygon', 'svg'), 'polygon',
                              inkex.addNS('ellipse', 'svg'), 'ellipse',
                              inkex.addNS('circle', 'svg'), 'circle']:
                # Replace this element with a <path> element

                if node in self.paths:
                    # Create a new <path> element
                    # We simply copy all of the attributes from
                    # the old element to this new element even though
                    # some of the attributes are no longer relevant
                    new_node = inkex.etree.Element(inkex.addNS('path', 'svg'), node.attrib)
                    new_node.set('d', self.paths[node][1:])

                    # Now replace the old element with this element
                    nodes[i] = new_node

                    # And dispose of the old data and element
                    del self.paths[node]
                    del node

            else:

                pass

    def recursivelyGetEnclosingTransform(self, node):

        """
        Determine the cumulative transform which node inherits from
        its chain of ancestors.
        """
        node = node.getparent()
        if node is not None:
            parent_transform = self.recursivelyGetEnclosingTransform(node)
            node_transform = node.get('transform', None)
            if node_transform is None:
                return parent_transform
            else:
                tr = parseTransform(node_transform)
                if parent_transform is None:
                    return tr
                else:
                    return composeTransform(parent_transform, tr)
        else:
            return self.docTransform

    def effect(self):

        # Viewbox handling
        self.handleViewBox()

        # Locate the center of the document by obtaining its dimensions
        if (self.docHeight is None) or (self.docWidth is None):
            inkex.errormsg('Document has inappropriate width or height units')
            return
        self.cy = self.docHeight / 2.0
        self.cx = self.docWidth / 2.0

        # First traverse the document (or selected items), reducing
        # everything to line segments.  If working on a selection,
        # then determine the selection's bounding box in the process.
        # (Actually, we just need to know it's extrema on the x-axis.)

        if self.options.ids:
            # Traverse the selected objects
            for id_ in self.options.ids:
                transform = self.recursivelyGetEnclosingTransform(self.selected[id_])
                self.recursivelyTraverseSvg([self.selected[id_]], transform, find_bbox=True)
            # Use as the vertical centerline the midpoint between
            # the bounding box's extremal X coordinates
            self.cx = 0.5 * (self.xmin + self.xmax)
        else:
            # Traverse the entire document building new, transformed paths
            self.recursivelyTraverseSvg(self.document.getroot(), self.docTransform)

        # Now that we know the x-axis extrema, we can remap the data
        # Had we know the x-axis extrema in advance (i.e., operating
        # on the entire document), then we could have done the mapping
        # at the same time we "rendered" everything to line segments.

        for key in self.paths:
            self.mapPathVertices(key)

        # And now replace the old paths with the new paths
        # WE DO NOT compute and replace the paths in the same pass!
        # So doing can cause multiple transformations of cloned paths

        self.recursivelyReplaceSvg(self.document.getroot(), self.docTransform)  # TODO The arguments here don't look right.


if __name__ == '__main__':

    e = Map()
    e.affect()
