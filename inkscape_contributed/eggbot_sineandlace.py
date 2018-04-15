#!/usr/bin/env python
# coding=utf-8

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

from math import pi, cos, sin

import inkex
import simplepath
import simplestyle

VERSION = 1


def parseDesc(str):
    """
    Create a dictionary from string description
    """

    if str is None:
        return {}
    else:
        return dict([tok.split(':') for tok in str.split(';') if len(tok)])


def formatDesc(d):
    """
    Format an inline name1:value1;name2:value2;... style attribute value
    from a dictionary
    """

    return ';'.join([atr + ':' + str(val) for atr, val in d.iteritems()])


def drawSine(cycles=8, rn=0, rm=0, nPoints=50, offset=None,
             height=200, width=3200, rescale=0.98, bound1='', bound2='', fun='sine', spline=True):
    """
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
    """

    """
    A complicated way of plotting y = sin(x)

    Complicated because we wish to generate the sine wave using a
    parametric representation.  For plotting a single sine wave in
    Cartesian coordinates, this is overkill.  However, it's useful
    for when we want to compress and expand the amplitude of the
    sine wave in accord with upper and lower boundaries which themselves
    are functions.  By parameterizing everything in sight with the
    same parameter s and restricting s to the range [0, 1], our life
    is made much easier.
    """
    if offset is None:
        offset = [0, 0]

    bounded = False

    if bound1 and bound2:

        func = parseDesc(bound1)
        if len(func) == 0:
            return None, None
        m1 = int(func['rm'])
        if m1 == 0:
            x_min1 = 0.0
        else:
            x_min1 = 2 * pi * float(func['rn']) / float(m1)
        x_max1 = x_min1 + 2 * pi * float(func['cycles'])
        y_min1 = -1.0
        y_max1 = 1.0
        y_scale1 = float(func['height']) / (y_max1 - y_min1)
        y_offset1 = float(func['y'])
        Y1s = lambda s: y_offset1 - y_scale1 * sin(x_min1 + (x_max1 - x_min1) * s)

        func = parseDesc(bound2)
        if len(func) == 0:
            return None, None
        m2 = int(func['rm'])
        if m2 == 0:
            x_min2 = 0.0
        else:
            x_min2 = 2 * pi * float(func['rn']) / float(m2)
        x_max2 = x_min2 + 2 * pi * float(func['cycles'])
        y_min2 = -1.0
        y_max2 = 1.0
        y_scale2 = float(func['height']) / (y_max2 - y_min2)
        y_offset2 = float(func['y'])
        Y2s = lambda s: y_offset2 - y_scale2 * sin(x_min2 + (x_max2 - x_min2) * s)

        bounded = True

    rescale = float(rescale)
    x_offset = float(offset[0])
    y_offset = float(offset[1])

    # Each cycle is 2pi
    r = 2 * pi * float(cycles)
    if (int(rm) == 0) or (int(rn) == 0):
        x_min = 0.0
    else:
        x_min = 2 * pi * float(rn) / float(rm)
    x_max = x_min + r
    x_scale = float(width) / r  # width / ( x_max - x_min )

    y_min = -1.0
    y_max = 1.0
    y_scale = float(height) / (y_max - y_min)

    # Our parametric equations which map the results to our drawing window
    # Note the "-y_scale" that's because in SVG, the y-axis runs "backwards"
    if not fun:
        fun = 'sine'
    fun = fun.lower()
    if fun == 'sine':
        Xs = lambda s: x_offset + x_scale * (x_max - x_min) * s
        Ys = lambda s: y_offset - y_scale * sin(x_min + (x_max - x_min) * s)
        dYdXs = lambda s: -y_scale * cos(x_min + (x_max - x_min) * s) / x_scale
    elif fun == 'lace':
        Xs = lambda s: x_offset + x_scale * ((x_max - x_min) * s + 2 * sin(2 * s * (x_max - x_min) + pi))
        dXs = lambda s: x_scale * (x_max - x_min) * (1.0 + 4.0 * cos(2 * s * (x_max - x_min) + pi))
        Ys = lambda s: y_offset - y_scale * sin(x_min + (x_max - x_min) * s)
        dYs = lambda s: -y_scale * cos(x_min + (x_max - x_min) * s) * (x_max - x_min)
        dYdXs = lambda s: dYs(s) / dXs(s)
    else:
        inkex.errormsg('Unknown function {0} specified'.format(fun))
        return

    # Derivatives: remember the chain rule....
    # dXs = lambda s: x_scale * ( x_max - x_min )
    # dYs = lambda s: -y_scale * cos( x_min + ( x_max - x_min ) * s ) * ( x_max - x_min )

    # x_third is 1/3 the step size
    nPoints = int(nPoints)

    # x_third is 1/3 the step size; note that Xs(1) - Xs(0) = x_scale * ( x_max - x_min )
    x_third = (Xs(1.0) - Xs(0.0)) / float(3 * (nPoints - 1))

    if bounded:
        y_upper = Y2s(0.0)
        y_lower = Y1s(0.0)
        y_offset = 0.5 * (y_upper + y_lower)
        y_upper = y_offset + rescale * (y_upper - y_offset)
        y_lower = y_offset + rescale * (y_lower - y_offset)
        y_scale = (y_upper - y_lower) / (y_max - y_min)

    x1 = Xs(0.0)
    y1 = Ys(0.0)
    dx1 = 1.0
    dy1 = dYdXs(0.0)

    # Starting point in the path is ( x, sin(x) )
    path_data = []
    path_data.append(['M ', [x1, y1]])

    for i in range(1, nPoints):

        s = float(i) / float(nPoints - 1)
        if bounded:
            y_upper = Y2s(s)
            y_lower = Y1s(s)
            y_offset = 0.5 * (y_upper + y_lower)
            y_upper = y_offset + rescale * (y_upper - y_offset)
            y_lower = y_offset + rescale * (y_lower - y_offset)
            y_scale = (y_upper - y_lower) / (y_max - y_min)

        x2 = Xs(s)
        y2 = Ys(s)
        dx2 = 1.0
        dy2 = dYdXs(s)
        if dy2 > 10.0:
            dy2 = 10.0
        elif dy2 < -10.0:
            dy2 = -10.0

        # Add another segment to the plot
        if spline:
            path_data.append([' C ',
                             [x1 + (dx1 * x_third),
                              y1 + (dy1 * x_third),
                              x2 - (dx2 * x_third),
                              y2 - (dy2 * x_third),
                              x2, y2]])
        else:
            path_data.append([' L ', [x1, y1]])
            path_data.append([' L ', [x2, y2]])
        x1 = x2
        y1 = y2
        dx1 = dx2
        dy1 = dy2

    path_desc = \
        'version:{0:d};style:linear;function:sin(x);'.format(VERSION) + \
        'cycles:{0:f};rn:{1:d};rm:{2:d};points:{3:d};'.format(cycles, rn, rm, nPoints) + \
        'width:{0:d};height:{1:d};x:{2:d};y:{3:d}'.format(width, height, offset[0], offset[1])

    return path_data, path_desc


class SpiroSine(inkex.Effect):
    nsURI = 'http://sample.com/ns'
    nsPrefix = 'doof'

    def __init__(self):

        inkex.Effect.__init__(self)

        self.OptionParser.add_option("--tab",  # NOTE: value is not used.
                                     action="store", type="string",
                                     dest="tab", default="splash",
                                     help="The active tab when Apply was pressed")

        self.OptionParser.add_option('--fCycles', dest='fCycles',
                                     type='float', default=10.0, action='store',
                                     help='Number of cycles (periods)')

        self.OptionParser.add_option('--nrN', dest='nrN',
                                     type='int', default=0, action='store',
                                     help='Start x at 2 * pi * n / m')

        self.OptionParser.add_option('--nrM', dest='nrM',
                                     type='int', default=0, action='store',
                                     help='Start x at 2 * pi * n / m')

        self.OptionParser.add_option('--fRecess', dest='fRecess',
                                     type='float', default=2.0, action='store',
                                     help='Recede from envelope by factor')

        self.OptionParser.add_option("--nSamples", dest="nSamples",
                                     type="int", default=50.0, action="store",
                                     help="Number of points to sample")

        self.OptionParser.add_option("--nWidth", dest="nWidth",
                                     type="int", default=3200, action="store",
                                     help="Width in pixels")

        self.OptionParser.add_option("--nHeight", dest="nHeight",
                                     type="int", default=100, action="store",
                                     help="Height in pixels")

        self.OptionParser.add_option("--nOffsetX", dest="nOffsetX",
                                     type="int", default=0, action="store",
                                     help="Starting x coordinate (pixels)")

        self.OptionParser.add_option("--nOffsetY", dest="nOffsetY",
                                     type="int", default=400, action="store",
                                     help="Starting y coordinate (pixels)")

        self.OptionParser.add_option('--bLace', dest='bLace',
                                     type='inkbool', default=False, action='store',
                                     help='Lace')

        self.OptionParser.add_option('--bSpline', dest='bSpline',
                                     type='inkbool', default=True, action='store',
                                     help='Spline')

        self.recess = 0.95

    def effect(self):

        inkex.NSS[self.nsPrefix] = self.nsURI

        if self.options.bLace:
            func = 'lace'
        else:
            func = 'sine'

        f_recess = 1.0
        if self.options.fRecess > 0.0:
            f_recess = 1.0 - self.options.fRecess / 100.0
            if f_recess <= 0.0:
                f_recess = 0.0

        if self.options.ids:
            if len(self.options.ids) == 2:
                desc1 = self.selected[self.options.ids[0]].get(inkex.addNS('desc', self.nsPrefix))
                desc2 = self.selected[self.options.ids[1]].get(inkex.addNS('desc', self.nsPrefix))
                if (not desc1) or (not desc2):
                    inkex.errormsg('Selected objects do not smell right')
                    return
                path_data, path_desc = drawSine(self.options.fCycles,
                                                self.options.nrN,
                                                self.options.nrM,
                                                self.options.nSamples,
                                                [self.options.nOffsetX, self.options.nOffsetY],
                                                self.options.nHeight,
                                                self.options.nWidth,
                                                f_recess,
                                                desc1, desc2, func, self.options.bSpline)
            else:
                inkex.errormsg('Exactly two objects must be selected')
                return
        else:
            self.document.getroot().set(inkex.addNS(self.nsPrefix, 'xmlns'), self.nsURI)

            path_data, path_desc = drawSine(self.options.fCycles,
                                            self.options.nrN,
                                            self.options.nrM,
                                            self.options.nSamples,
                                            [self.options.nOffsetX, self.options.nOffsetY],
                                            self.options.nHeight,
                                            self.options.nWidth,
                                            f_recess,
                                            '',
                                            '',
                                            func,
                                            self.options.bSpline)

        style = {'stroke': 'black', 'stroke-width': '1', 'fill': 'none'}
        path_attrs = {
            'style': simplestyle.formatStyle(style),
            'd': simplepath.formatPath(path_data),
            inkex.addNS('desc', self.nsPrefix): path_desc}
        newpath = inkex.etree.SubElement(self.document.getroot(),
                                         inkex.addNS('path', 'svg '), path_attrs, nsmap=inkex.NSS)


if __name__ == '__main__':

    e = SpiroSine()
    e.affect()
