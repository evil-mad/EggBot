#!/usr/bin/env python

# Adapted from generic template by Tavmjong Bah

import inkex
import re

class C(inkex.Effect):
  def __init__(self):
    inkex.Effect.__init__(self)
    self.OptionParser.add_option("-w", "--width",  action="store", type="int",    dest="generic_width",  default="1920", help="Custom width")
    self.OptionParser.add_option("-z", "--height", action="store", type="int",    dest="generic_height", default="1080", help="Custom height")

  def effect(self):

    width  = self.options.generic_width
    height = self.options.generic_height
    unit   = "px"

    root = self.document.getroot()
    root.set("id", "SVGRoot")
    root.set("width",  str(width) + unit)
    root.set("height", str(height) + unit)
    root.set("viewBox", "0 0 " + str(width) + " " + str(height) )

    namedview = root.find(inkex.addNS('namedview', 'sodipodi'))
    if namedview is None:
        namedview = inkex.etree.SubElement( root, inkex.addNS('namedview', 'sodipodi') );

    namedview.set(inkex.addNS('document-units', 'inkscape'), unit)

    # Until units are supported in 'cx', etc.
    namedview.set(inkex.addNS('zoom', 'inkscape'), str(512.0/self.uutounit( width,  'px' )) )
    namedview.set(inkex.addNS('cx',   'inkscape'), str(self.uutounit( width,  'px' )/2.0 ) )
    namedview.set(inkex.addNS('cy',   'inkscape'), str(self.uutounit( height, 'px' )/2.0 ) )

    namedview.set( 'pagecolor', "#ffffff" )
    namedview.set( 'bordercolor', "#666666" )
    namedview.set(inkex.addNS('pageopacity', 'inkscape'), "1.0" )
    namedview.set(inkex.addNS('pageshadow', 'inkscape'), "0" )
    

c = C()
c.affect()
