#!/usr/bin/env python

# Post Process a bitmap image traced with Inkscape's Trace Bitmap tool
# The output of Trace Bitmap is traversed and each <path> is put into
# an Inkscape layer whose name is Eggbot friendly.  Owing to how the
# Trace Bitmap tool operates in Inkscape 0.48, all the traced regions
# of a given "scanned" color are put into a single <path>.  This makes
# it easy to put all the traced regions of a single color into a single
# layer: just put each <path> into its own layer.

# Written by Daniel C. Newman ( dan dot newman at mtbaldy dot us )
# 9 October 2010

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
import gettext
import simplestyle

class EggBot_PostProcessTraceBitmap( inkex.Effect ):

	def __init__(self):
		inkex.Effect.__init__( self )
		self.OptionParser.add_option(
			"--outlineRegions", action="store", dest="outlineRegions",
			type="inkbool", default=True,
			help="Outline the regions with a stroked line of the same color as the region itself" )
		self.OptionParser.add_option(
			"--fillRegions", action="store", dest="fillRegions",
			type="inkbool", default=True,
			help="Fill regions with color" )
		self.OptionParser.add_option(
			"--removeImage", action="store", dest="removeImage",
			type="inkbool", default=True,
			help="Remove the traced bitmap image from the drawing" )

	def effect(self):

		root = self.document.getroot()

		count = 0
		for path in self.document.xpath( '//svg:path', namespaces=inkex.NSS ):

			# Default settings for now
			stroke, fill, color = ( 'none', 'none', 'unknown' )

			# Get the paths style attribute
			style = simplestyle.parseStyle( path.get( 'style', '' ) )
			# Obtain the fill color from the path's style attribute
			if 'fill' in style:
				color = style['fill']
				if self.options.fillRegions:
					fill = color
				if self.options.outlineRegions:
					stroke = color

			# Now add or change the fill color in the path's style
			style['fill'] = fill

			# Add or change the stroke behavior in the path's style
			style['stroke'] = stroke

			# And change the style attribute for the path
			path.set( 'style', simplestyle.formatStyle( style ) )

			# Create a group <g> element under the document root
			layer = inkex.etree.SubElement( root, inkex.addNS( 'g', 'svg' ) )

			# Add Inkscape layer attributes to this new group
			count += 1
			layer.set( inkex.addNS('groupmode', 'inkscape' ), 'layer' )
			layer.set( inkex.addNS( 'label', 'inkscape' ), '%d - %s' % ( count, color ) )

			# Now move this path from where it was to being a child
			# of this new group/layer we just made
			layer.append( path )

		# Remove any image
		# For color scans, Trace Bitmap seems to put the
		# image in the same layer & group as the traced regions.
		# BUT, for gray scans, it seems to leave the image by
		# itself as a child of the root document

		if self.options.removeImage:
			for node in self.document.xpath( '//svg:image', namespaces=inkex.NSS ):
				parent = node.getparent()
				if ( parent.tag == 'svg' ) or \
				   ( parent.tag == inkex.addNS( 'svg', 'svg' ) ):
					parent.remove( node )
				else:
					gparent = parent.getparent()
					try:
						gparent.remove( parent )
					except:
						parent.remove( node)

		inkex.errormsg( gettext.gettext( 'Finished.  Created %d layers' ) % count )

if __name__ == '__main__':
	e = EggBot_PostProcessTraceBitmap()
	e.affect()
