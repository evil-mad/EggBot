#!/usr/bin/env python

# Adapted from generic template by Tavmjong Bah

import inkex

class EmptyEggBot(inkex.EffectExtension):

    def __init__(self):
        super(EmptyEggBot, self).__init__()
        self.arg_parser.add_argument("-w", "--width", type=int, dest="generic_width", default="3200", help="Custom width")
        self.arg_parser.add_argument("-z", "--height", type=int, dest="generic_height", default="800", help="Custom height")

    def effect(self):

        width  = self.options.generic_width
        height = self.options.generic_height
    
        root = self.document.getroot()
        root.set("id", "SVGRoot")
        root.set("width",  str(width) + 'px')
        root.set("height", str(height) + 'px')
        root.set("viewBox", "0 0 " + str(width) + " " + str(height) )

        namedview = self.svg.namedview
        namedview.set(inkex.addNS('document-units', 'inkscape'), 'px')

        namedview.set(inkex.addNS('zoom', 'inkscape'), str(512.0 / width) )

        namedview.set(inkex.addNS('cx', 'inkscape'), str(width / 2.0))
        namedview.set(inkex.addNS('cy', 'inkscape'), str(height / 2.0))


if __name__ == '__main__':
    EmptyEggBot().run()
