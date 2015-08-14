# eggbot_acrostic.py
#
# Render an acrostic poem using the Hershey fonts
#
# This extension requires the hersheydata.py file which is part of the
# Hershey Text rendering extension written by Windell H. Oskay of
# www.evilmadscientist.com.  Information on that extension may be found at
#
#   http://www.evilmadscientist.com/go/hershey
#
# Copyright 2011, Daniel C. Newman,
#
# Significant portions of this code were written by Windell H. Oskay and are
# Copyright 2011, Windell H. Oskay, www.evilmadscientist.com
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

import hersheydata			# data file w/ Hershey font data
import inkex
import simplestyle

WIDTH     = 3200
HEIGHT    = 800
MAX_H     = 32		# Maximum height of a Hershey font character (parens)
LINE_SKIP = 6		# Baseline skip between lines of text

# Mapping table to map the names used here to the corresponding
# names used in hersheydata.py.  This helps prevent end users from
# being impacted by a name change in hersheydata.py.  This can also
# be used to deal with a face being removed from hersheydata.py

map_our_names_to_hersheydata = {
	'astrology' : 'astrology',
	'cursive' : 'cursive',
	'cyrillic' : 'cyrillic',
	'futural' : 'futural',
	'futuram' : 'futuram',
	'gothiceng' : 'gothiceng',
	'gothicger' : 'gothicger',
	'gothicita' : 'gothicita',
	'greek' : 'greek',
	'japanese' : 'japanese',
	'markers' : 'markers',
	'mathlow' : 'mathlow',
	'mathupp' : 'mathupp',
	'meteorology' : 'meteorology',
	'music' : 'music',
	'scriptc' : 'scriptc',
	'scripts' : 'scripts',
	'symbolic' : 'symbolic',
	'timesg' : 'timesg',
	'timesi' : 'timesi',
	'timesib' : 'timesib',
	'timesr' : 'timesr',
	'timesrb' : 'timesrb' }

# The following two routines are lifted with impunity from Windell H. Oskay's
# hershey.py Hershey Text extension for Inkscape.  They are,
# Copyright 2011, Windell H. Oskay, www.evilmadscientist.com

def draw_svg_text(char, face, offset, vertoffset, parent):

	style = { 'stroke': '#000000', 'fill': 'none' }
	pathString = face[char]
	splitString = pathString.split()
	midpoint = offset - int(splitString[0])
	i = pathString.find("M")
	if i >= 0:
		pathString = pathString[i:] #portion after first move
		trans = 'translate(' + str(midpoint) + ',' + str(vertoffset) + ')'
		text_attribs = {'style':simplestyle.formatStyle(style), 'd':pathString, 'transform':trans}
		inkex.etree.SubElement(parent, inkex.addNS('path','svg'), text_attribs)
	return midpoint + int(splitString[1]) 	#new offset value

def renderText( parent, w, y, text, typeface ):

	'''
	Render a string of text starting from the point (w, y) and using
	the supplied typeface data.
	'''

	if ( text is None ) or ( text == '' ):
		return

	spacing = 3  # spacing between letters
	letterVals = [ ord( q ) - 32 for q in text ]

	for q in letterVals:
		if ( q < 0 ) or ( q > 95 ):
			w += 2 * spacing
		else:
			w = draw_svg_text( q, typeface, w, y, parent )

	return w

def renderLine( parent, x, y, line, typeface1, typeface2 ):

	'''
	Render a single line of text:
	+ The text runs horizontally from left to right starting at the point (x,y)
	+ The first character of the line is written using "typeface1"
	+ The remainder of the line of text is written using "typeface2"

	The entire line is stored as individual paths which are child elements
	of the SVG element "parent".  The text in typeface2 (line[1:]) is also
	placed in it's own subgroup.  The leading character is not placed in
	that subgroup.  The reasoning is that the user may want to pick out
	the first character of each line of text and put them in another layer
	for plotting in a different color.
	'''

	# Return now if there's nothing to do
	if ( line is None ) or ( line == '' ):
		return

	# Render the first character
	x = renderText( parent, x, y, [ line[0] ], typeface1 )

	# Render the rest of the line
	line = line[1:]
	if line == '':
		return
	g = inkex.etree.SubElement( parent, 'g' )
	renderText( g, x, y, line, typeface2 )

class AcrosticText( inkex.Effect ):

	def __init__( self ):

		inkex.Effect.__init__( self )
		self.OptionParser.add_option( "--tab",	#NOTE: value is not used.
			action="store", type="string", dest="tab", default="splash",
			help="The active tab when Apply was pressed" )
		self.OptionParser.add_option( "--line01", action="store",
			type="string", dest="line1", default="")
		self.OptionParser.add_option( "--line02", action="store",
			type="string", dest="line2", default="")
		self.OptionParser.add_option( "--line03", action="store",
			type="string", dest="line3", default="")
		self.OptionParser.add_option( "--line04", action="store",
			type="string", dest="line4", default="")
		self.OptionParser.add_option( "--line05", action="store",
			type="string", dest="line5", default="")
		self.OptionParser.add_option( "--line06", action="store",
			type="string", dest="line6", default="")
		self.OptionParser.add_option( "--line07", action="store",
			type="string", dest="line7", default="")
		self.OptionParser.add_option( "--line08", action="store",
			type="string", dest="line8", default="")
		self.OptionParser.add_option( "--line09", action="store",
			type="string", dest="line9", default="")
		self.OptionParser.add_option( "--line10", action="store",
			type="string", dest="line10", default="")
		self.OptionParser.add_option( "--line11", action="store",
			type="string", dest="line11", default="")
		self.OptionParser.add_option( "--line12", action="store",
			type="string", dest="line12", default="")
		self.OptionParser.add_option( "--face1",
			action="store", type="string", dest="face1", default="scriptc",
			help="Leading font typeface" )
		self.OptionParser.add_option( "--face2", action="store",
			type="string", dest="face2", default="scripts",
			help="Secondary typeface" )
		self.OptionParser.add_option( "--flip", action="store", type="inkbool",
			dest="flip", default=False,
			help="Flip the text for plotting with the egg's bottom at the egg motor" )
		self.OptionParser.add_option( "--stretch",
			action="store", type="inkbool", dest="stretch", default=True,
			help="Stretch the text horizontally to account for egg distortions" )

	def effect( self ):

		# Process the lines, ignoring leading or trailing blank lines
		# and collapsing multiple internal runs of blank lines into a
		# single blank line.
		lines = []
		prior_empty = False
		for i in range( 1, 13 ):
			line = eval( 'self.options.line' + str( i ) ).strip()
			if line == '':
				if len( lines ) != 0:
					prior_empty = True
			else:
				if prior_empty:
					lines.append( '' )
					prior_empty = False
				lines.append( line )

		# Return now if there are no lines to print
		line_count = len( lines )
		if line_count == 0:
			return

		# Determine how much vertical room we need for our text
		h = line_count * MAX_H + ( line_count - 1 ) * LINE_SKIP

		svg = self.document.getroot()
		doc_height = self.unittouu( svg.attrib['height'] )
		if doc_height <= 0:
			doc_height = HEIGHT
		doc_width = self.unittouu( svg.attrib['width'] )
		if doc_width <= 0:
			doc_width = WIDTH

		# Scale to doc_height pixels high
		scale_y = float( doc_height ) / float( h )
		if self.options.stretch:
			scale_x = scale_y * 1.5
		else:
			scale_x = scale_y

		# Determine where to position the text
		# We do not bother centering the text horizontally
		# to do that we would need to pre-render the text to determine
		# the length of the longest line.  That's too much bother so
		# we just skip that potential nice-to-have.
		x = float( doc_width ) / ( 2.0 * scale_x )
		y = float( MAX_H ) / scale_y

		# Get the two type faces
		name1 = self.options.face1
		if map_our_names_to_hersheydata.has_key( name1 ):
			name1 = map_our_names_to_hersheydata[name1]
		face1 = eval( 'hersheydata.' + name1 )
		name2 = self.options.face2
		if map_our_names_to_hersheydata.has_key( name2 ):
			name2 = map_our_names_to_hersheydata[name2]
		face2 = eval( 'hersheydata.' + name2 )

		# Create the group which will contain all of the text
		# We DO NOT make this a child of the current layer as that
		# would subject us to any transforms it might have.  That's
		# an issue even in the simple case of someone opening a default
		# document and then changing its dimensions to 3200 x 800:
		# Inkscape imposes a transform in that situation.  While that
		# transform is no big deal, it's another complication in trying
		# to just make the resulting text look right (right size, right
		# approximate position, etc.).

		if self.options.flip:
			attribs = { 'transform' : 'matrix(-%f,0,0,-%f,%d,%d)' % ( scale_x, scale_y, doc_width, doc_height ) }
		else:
			attribs = { 'transform' : 'scale(%f,%f)' % ( scale_x, scale_y ) }
		container = inkex.etree.SubElement( self.document.getroot(), 'g', attribs )

		# Finally, we render each line of text
		for i in range( 0, len( lines ) ):
			if lines[i] != '':
				g = inkex.etree.SubElement( container, 'g' )
				renderLine( g, x, y, lines[i], face1, face2 )
			y += MAX_H + LINE_SKIP

if __name__ == '__main__':
	e = AcrosticText()
	e.affect()
