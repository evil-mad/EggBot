# eggbot_spiraltext.py
#
# Render a passage of text using the Hershey fonts, then stretch it so
# that it will wrap multiple times around an egg, and finally tilt it
# so that it will spiral as it wraps.
#
# + The wrapping need not be an integral multiple of 3200 pixels
#
# + The text tilt is computed to use the full height of the document
#
# + The text can be run starting from the top of the page or from
#   the bottom (and upside down).  This latter orientation is useful
#   when placing the bottom of the egg (fat end) in the egg motor's
#   egg cup
#
# + The text can be stretched more horizontally than vertically to
#   compensate for some of the geometry issues associated with drawing
#   on eggs.
#
# + The text can contain markup (see below)
#
# This extension also permits some basic markup of the passage using
# XHTML-like conventions and a limited set of tags:
#
#  <sams>   - A simple typeface which lacks serifs
#  <times>  - "Times" like typeface (a face with serifs)
#  <script> - A flowing script font
#  <b>      - Boldface
#  <em>     - Emphasis
#  <i>      - Italics
#  <face>   - Where "face" is any of the typeface names from hersheydata.py
#
# The markup processing is not XML-conformant: we don't expect a well-formed
# document as input.  No single root element is required.  And, at the end of
# the text, we do not require closure of any open tags.  We do however enforce
# proper nesting of tags: an element cannot be closed unless its children have
# already been closed.  This is more to prevent ambiguity about whether or
# not closing a typeface also closes any markup operating under it (e.g.,
# does <sans><b>text</sans> mean that the <b> was implicitly ended when
# </sans> was encountered?
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
# Small portions of this code were changed by Sheldon B. Michaels 2016,
# in order to accomodate the addition of several new faces
# (the "EMS" series) to hersheydata.py.  Additionally, changes were made
# to the default text rendering style.
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

import sys
import hersheydata			#data file w/ Hershey font data
import inkex
import simplestyle
import math
import string

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
	style = { 'stroke' : '#000000', 'fill' : 'none', 'stroke-linecap' : 'round', 'stroke-linejoin' : 'round' }
		# Apply rounding to ends so that user gets best impression of final printed text appearance.
	pathString = face[char]
	splitString = pathString.split()
	midpoint = offset - float(splitString[0])
	i = pathString.find("M")
	if i >= 0:
		pathString = pathString[i:] #portion after first move
		trans = 'translate(' + str(midpoint) + ',' + str(vertoffset) + ')'
		text_attribs = {'style':simplestyle.formatStyle(style), 'd':pathString, 'transform':trans}
		inkex.etree.SubElement(parent, inkex.addNS('path','svg'), text_attribs)
	return midpoint + float(splitString[1]) 	#new offset value

def renderText( parent, markup ):
	# Embed text in group to make manipulation easier:
	g_attribs = {inkex.addNS('label','inkscape'):'Hershey Text' }
	g = inkex.etree.SubElement(parent, 'g', g_attribs)

	w = 0  #Initial spacing offset
	spacing = 3  # spacing between letters

	for Face, Text in markup:
		if map_our_names_to_hersheydata.has_key(Face):
			Face = map_our_names_to_hersheydata[Face]
		font = eval('hersheydata.' + Face)
		letterVals = [ord(q) - 32 for q in Text]
		for q in letterVals:
			if (q < 0) or (q > 95):
				w += 2*spacing
			else:
				w = draw_svg_text(q, font, w, 0, g)
	return g, w

# The generic font "families" we support
generic_families = ( 'sans', 'script', 'times' )

# Convert "family-name" + "bold-0-or-1" + "italic-0-or-1" to a typeface name
family_to_font = {
	'sans00' : 'futural', 'sans10' : 'futuram', 'sans01' : 'futural', 'sans11' : 'futuram',
	'times00' : 'timesr', 'times10' : 'timesrb', 'times01' : 'timesi', 'times11' : 'timesib',
	'script00' : 'scripts', 'script10' : 'scriptc', 'script01' : 'scripts', 'script11' : 'scriptc' }
emphasis_is_bold = { 'sans' : True, 'times' : False, 'script' : True }

# Short list of entity references
entity_refs = { '&lt;' : '<', '&gt;' : '>', '&amp;' : '&', '&quot;' : '"', '&apos' : "'", '&nbsp;' : ' ' }

def normalize_possible_EMS_string( tag ):
	# Normalizes tag name by removing any spaces
	sNormalizedTag = string.replace( tag, ' ', '')
	return sNormalizedTag

def is_valid_EMS_name( tag ):
	# returns true if family is one of the "EMS" faces in hersheydata.py
	# else false
	sNormalizedTag = normalize_possible_EMS_string( tag )
	bRetVal = False		# default assumption
	try:
		fontgroup = hersheydata.group_allfonts
	except:
		# User probably has old version of hersheydata.py
		pass
	else:
		for f in fontgroup:
			if f[0] == sNormalizedTag:
				bRetVal = True
				break
			
	return bRetVal

def pickFace( family, bold=False, italics=False, emphasis=False ):

	if ( family is None ) or ( family == '' ):
		return None

	b = '0'
	i = '0'

	# If using a generic font family, then determine how to map <em>
	if emphasis and ( family in generic_families ):
		if emphasis_is_bold[family]:
			bold = True
		else:
			italics = True

	if bold:
		b = '1'

	if italics:
		i = '1'

	if family_to_font.has_key( family + b + i ):
		return family_to_font[family + b + i]

	return family

def processMarkup( text, family='sans' ):
	if ( text is None ):
		text = ''

	# By default we assume 'sans'
	if ( family is None ) or ( family == ''):
		family = 'sans'
	family_default = family
	face_stack = [ family ]

	# Bold and italics off
	bold = False
	emphasis = False
	italic = False

	# Set the current typeface
	face = pickFace( family, bold, italic, emphasis )

	# And the result of markup processing so far
	markup = []

	# We keep a queue / list of the open markup tags
	# When a tag is closed, we expect it to be well nested.  To enforce
	# that expectation, we make sure that we are closing the most recently
	# opened tag.  While this may seem overly picky, it's easier than worrying
	# issues like, "Does closing a typeface imply implicitly closing <b> or <it>?"
	# And, "Does starting a new typeface imply closing the prior selected face?"
	tags_used = []

	outstr = ''
	i = 0
	while i < len( text ):
		# An entity reference?
		if text[i] == '&':
			j = text.find( ';', i+1 )
			if ( j != -1 ):
				eref = text[i:j+1]
				if entity_refs.has_key(eref):
					outstr += entity_refs[eref]
					i = j + 1
				else:
					inkex.errormsg( 'Ignoring the unrecognized entity reference %s.' % eref )
					outstr += eref
					i = j + 1
			else:
				inkex.errormsg( 'An unescaped "&" was encountered; please replace it with "&amp;".' )
				break

		# Start of a tag (start-tag or end-tag? self-closing tags not supported)
		elif text[i] == '<':
			j = text.find( '>', i+1 )
			if ( j != -1 ) and ( j > ( i + 1) ):

				tag = text[i+1:j]
				i = j + 1

				if tag[0] == '/':
					# This is an end-tag (closing tag)
					close = True
					tag = tag[1:]

					# Ensure that the most recently opened tag is that which we are closing here
					# We'll pop the most recent tag from the queue of opened tags and see if
					# it matches
					if len( tags_used ) == 0:
						inkex.errormsg( 'The ending tag </%s> appeared before any start tag <%s>.' % ( tag, tag ) )
						break
					else:
						old_tag = tags_used.pop()
						if old_tag != tag:
							inkex.errormsg( 'The ending tag </%s> does not appear to be correctly nested; it tried to close the tag <%s>.  Sorry, but all tags must be properly nested.' % ( tag, old_tag ) )
							break
				else:
					# Start tag (opening tag)
					# Push it onto the queue of opened tags
					close = False
					tags_used.append( tag )

				if ( tag == 'b' ) or ( tag == 'strong' ):
					if bold == close:
						# Push prior string and font onto the stack
						if outstr != '':
							markup.append( [ face, outstr ] )
							outstr = ''

						# Start a new boldface string
						bold = not bold
						face = pickFace( family, bold, italic, emphasis )

				elif tag == 'i':
					if italic == close:
						# Push the prior string and font unto the stack
						if outstr != '':
							markup.append( [ face, outstr ] )
							outstr = ''

						# Start a new italicized string
						italic = not italic
						face = pickFace( family, bold, italic, emphasis )

				elif tag == 'em':
					if emphasis == close:
						# Push the prior string and font unto the stack
						if outstr != '':
							markup.append( [ face, outstr ] )
							outstr = ''

						# Start a new italicized string
						emphasis = not emphasis
						face = pickFace( family, bold, italic, emphasis )

				else:
					bValidEMSName = is_valid_EMS_name( tag )
					if bValidEMSName:
						tag = normalize_possible_EMS_string( tag )
					if (
							( tag not in generic_families )
							and ( not map_our_names_to_hersheydata.has_key( tag ) )
							and ( not bValidEMSName )
						):
						if close:
							inkex.errormsg( 'Ignoring the unrecognized tag </%s>.' % tag )
						else:
							inkex.errormsg( 'Ignoring the unrecognized tag <%s>.' % tag )
					else:
						if outstr != '':
							markup.append( [face, outstr] )
							outstr = ''
						if not close:
							family = tag
							face_stack.append( family )
						else:
							if len( face_stack ) > 0:
								# Current face on the stack should be the one we just closed
								face_stack.pop()
								if len( face_stack ) > 0:
									family = face_stack[len( face_stack) - 1]
								else:
									family = default_family
							else:
								family = default_family
						face = pickFace( family, bold, italic, emphasis )
			else:
				inkex.errormsg( 'Ignoring unescaped "<"' )
				outstr += '<'
				i += 1
		else:
			outstr += text[i]
			i += 1

	# We won't worry about unclosed tags -- we're not trying to be an XML or XHTML parser

	# See if there was a hard error
	if i < len( text):
		return None

	# And push the last text into the list of processed markup
	if outstr != '':
		markup.append( [face, outstr] )

	return markup

class SpiralText( inkex.Effect ):

	def __init__( self ):
		inkex.Effect.__init__( self )
		self.OptionParser.add_option( "--tab",	#NOTE: value is not used.
			action="store", type="string",
			dest="tab", default="splash",
			help="The active tab when Apply was pressed" )
		self.OptionParser.add_option( "--text",
			action="store", type="string",
			dest="text", default="Hershey Text for Inkscape",
			help="The input text to render")
		self.OptionParser.add_option( "--fontfamily",
			action="store", type="string",
			dest="fontfamily", default="sans",
			help="The selected font face when Apply was pressed" )
		self.OptionParser.add_option( "--wrap",
			action="store", type="float",
			dest="wrap", default=float(10),
			help="Number of times to wrap the text around the egg" )
		self.OptionParser.add_option( "--flip",
			action="store", type="inkbool",
			dest="flip", default=False,
			help="Flip the text for plotting with the egg's bottom at the egg motor" )
		self.OptionParser.add_option( "--stretch",
			action="store", type="inkbool",
			dest="stretch", default=True,
			help="Stretch the text horizontally to account for egg distortions" )

	def effect( self ):

		markup = processMarkup( self.options.text, self.options.fontfamily )
		g,w = renderText( self.current_layer, markup )

		# Now to wrap the text N times around the egg, we need to scale it to have
		# length 3200 * N.  It's current width is w so the scale factor is (3200 * N) / w.

		scale_x = float( 3200 * self.options.wrap ) / float( w )
		scale_y = scale_x
		if self.options.stretch:
			scale_y = scale_y * 2.0 / 3.0

		# In planning the scaling, we'd like to know the height of our line of text.
		# Rather than computing its bounding box, we'll just use the height of the
		# parens from the Simplex Roman font.  And, we could compute that but we'll
		# just use our prior knowledge of it being 32.

		h = float( 32.0 )

		# And the angular tilt will be arcsine( height / (3200 * fWrap) )
		svg = self.document.getroot()
		height = float( self.unittouu( svg.attrib['height'] ) ) - h * scale_y
		angle = ( float( 180 ) / math.pi ) * \
			math.asin( height / float( 3200 * self.options.wrap ) )

		if self.options.flip:
			angle += float( 180.0 )
			t = 'translate(%f,%f) rotate(%f,%f,0) scale(%f,%f)' % ( -w*scale_x, h*scale_y, angle,
										 w*scale_x, scale_x, scale_y )
		else:
			t = 'translate(0,%f) rotate(%f,0,0) scale(%f,%f)' % ( h, angle, scale_x, scale_y )
		g.set( 'transform', t)

if __name__ == '__main__':
	e = SpiralText()
	e.affect()
