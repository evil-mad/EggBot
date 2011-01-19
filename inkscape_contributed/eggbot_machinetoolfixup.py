#!/usr/bin/env python

# This extension is intended to alter text from the Machine Tool font
# family to be suitable for Eggbot plotting.  It will
#
#   -- Remove the fill option
#   -- Enable the stroke option
#   -- Set the stroke color to that of the fill or to black if there
#        was no fill or the fill was white
#   -- Set the stroke width to 1 pixel if no stroke width was set or
#        if the stroke width was zero
#   -- Fixup the character paths to remove closing line segments which
#        existed in the character for purposes of making a closed polygon
#        which could then be filled
#
# Note that some of the characters in the Machine Tool font family ARE NOT
# single stroke characters and thus will not work well with this extension.
# For example, the @, %, and ! characters.

# Written by Daniel C. Newman ( dan dot newman at mtbaldy dot us )
# 19 January 2011

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
import simplepath
import simplestyle

class EggbotMachineToolFixup( inkex.Effect ):

	def __init__( self ):

		inkex.Effect.__init__( self )

	def fixstyle( self, node ):

		'''
		Fixup the style

		  - Disable fill
		  - Enable stroke
		  - Set stroke-width to 1 if not already set
		  - Remove font name info so that we don't process this again
		'''

		if not node:
			return

		style_str = node.get( 'style' )
		if style_str:
			style = simplestyle.parseStyle( style_str )
			del style['font-family']
			del style['-inkscape-font-specification']
			if 'fill' in style:
				f = style['fill']
				if ( f != 'none' ) and ( f != '#ffffff' ) and ( f != 'white' ):
					style['stroke'] = f
				else:
					style['stroke'] = '#000000'
			else:
				style['stroke'] = 'none'
			if 'stroke-width' in style:
				sw = float( style['stroke-width'] )
				if sw <= float( 1.0E-6 ):
					style['stroke-width'] = '1'
			else:
				style['stroke-width'] = '1'
			style['fill'] = 'none'
		else:
			# We should never end up here...
			style = { 'fill':'none', 'stroke':'#000000', 'stroke-width':'1' }
		style_str = simplestyle.formatStyle( style )
		node.set( 'style', style_str )

	def fixpaths( self, nodes ):

		if nodes is None:
			return

		for node in nodes:
			if ( node.tag != 'path' ) and ( node.tag != inkex.addNS( 'path', 'svg')):
				continue
			path_str = node.get( 'd' )
			if not path_str:
				continue

			# Parse the path data
			# This will convert all coordinates to absolute
			path = simplepath.parsePath( path_str )
			previous_path_elem = path[0]
			new_path = []
			for op in path[1:]:
				# Only add the prior element to the path
				# if this element is not the start of a new
				# line segment
				if op[0] != 'M':
					new_path.append( previous_path_elem )
				previous_path_elem = op

			# Do not add the last elem if it was a line-to op
			if previous_path_elem[0] != 'L':
				new_path.append( previous_path_elem )

			# Format the path data back into a string
			path_str = simplepath.formatPath( new_path )

			# Replace the old path data with this new path data
			node.set( 'd', path_str )

	def effect( self ):

		# Find all previously unaltered Machine Tool font characters
		doc = self.document.getroot()
		nodes = doc.xpath( '//svg:g[contains(@style,"font-family:Machine Tool")]',
				   namespaces=inkex.NSS )
		found = False
		if not nodes is None:
			for node in nodes:
				if node.tag == inkex.addNS( 'g', 'svg' ) or node.tag == 'g':
					found = True
					self.fixstyle( node )
					self.fixpaths( node )
		if not found:
			inkex.errormsg( gettext.gettext( 'Nothing found.  Did you remember to convert Object to Path?' ) )

if __name__ == '__main__':

	e = EggbotMachineToolFixup()
	e.affect()
