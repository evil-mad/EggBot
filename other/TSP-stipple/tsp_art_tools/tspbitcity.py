# tspbitcity.py
# 9/26/2010-A

# Python class which interprets the black bits in a black & white bitmap
# as the coordinates of "cities" on a map.  Turn these coordinates into a
# TSPLIB file for use as input to a TSP solver.

# Point output files from Adrian Secord's Weighted Voronoi Stippler are
# also recognized.  The coordinates from those files are floating point
# numbers and are rescaled to the range [0, 800] and converted to integers.
# The reason for this isn't for the TSP solver -- it handles floats just
# fine.  Rather, to have (1) have a consistent data type, and (2) the
# resulting SVG file is much smaller when integers are used as the
# coordinates.

# This file can also be run as a standalone program to produce a TSPLIB
# file from a bitmap:
#
#    python tspbitcity.py [input-bitmap-file [output-tsplib-file]]

# Written by Daniel C. Newman for the Eggbot Project
# dan dot newman at mtbaldy dot us
# 25 September 2010

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

import os
import sys

class tspBitCity:

	# When presented with a collection of floating point (x,y) coordinates,
	# we normalize their bounding box to have a height and width of BOXSIZE
	BOXSIZE = float( 800 )

	def __init__( self ):

		# We save the input bitmap file name for purposes of error reporting
		# and generating a default output file name

		self.infile = ''

		# Our width and height correspond to the size of the input bitmap
		# All coordinates (x, y) will satisfy 0 <= x < width and
		# 0 <= y < height

		self.width = 0
		self.height = 0

		# Our list of "city" (x, y) coordinates
		# Each member of the list is a 2-tuple (x, y) which satisfies
		# 0 <= x < width and 0 <= y < height
		#
		# Owing to the nature of our input bitmaps and the way we read them,
		# coordinate[i] = (x[i], y[i]) and coordinate[i+1] = (x[i+1], y[i+1])
		# will always satisfy
		#
		#     y[i] >= y[i+1]
		#
		# and if y[i] == y[i+1], then x[i] < x[i+1].  In other words, the
		# cities are sorted such that their y coordinates decrease as you
		# advance through the list of coordinates.

		self.coordinates = []

	# Load a PBM of type P4
	def __load_pbm_p4( self, f ):

		assert ( self.width > 0 ) and ( self.height > 0 )
		assert ( f )

		self.coordinates = []

		# PBM file goes from the top of the bitmap (y = h-1) to the
		# bottom of the bitmap (y = 0), and from the left of the bitmap
		# (x = 0) to the right of the bitmap (x = w)

		# Each line of the file contains w pixels with 8 pixels per byte
		# So, each line of the file must be (w + 7) >> 3 bytes long
		nbytes = ( self.width + 7 ) >> 3

		# Each line of the file from here on out corresponds to a
		# single row of the bitmap

		for row in range( self.height - 1, -1, -1 ):

			# Read the bitmap row
			row_bytes = f.read( nbytes )

			# Perform a sanity check
			if ( row_bytes == '' ) or ( row_bytes == '\n' ):
				sys.stderr.write( '1 Premature end-of-data encountered in %s\n' % self.infile )
				return False

			# And start at the first byte of the line read
			column_byte_index = 0

			# Convert the unsigned char byte to an integer
			column_byte = int( ord( row_bytes[0] ) )

			# Now process this row from left to right, x = 0 to x = w - 1
			pixel_mask = int( 0x80 )
			for column in range(0, self.width):

				# See if this bit is lit
				if pixel_mask & column_byte:
					# Bit is lit, save the coordinate of this pixel
					self.coordinates.append( ( column, row ) )

				# Now move our bitmask bit to the right by one pixel
				pixel_mask >>= 1

				# See if it's time to move to the next byte in the input line
				if pixel_mask == 0x00:
					column_byte_index += 1
					if column_byte_index < nbytes:
						column_byte = ord( row_bytes[column_byte_index] )
						pixel_mask = int( 0x80 )
					elif column < ( self.width - 1 ):
						# Something has gone wrong: we didn't read enough bytes?
						sys.stderr.write( '2 Premature end-of-file encountered in %s\n' % self.infile )
						return False

		return True

	# Load a PBM of type P1
	def __load_pbm_p1( self, f ):

		assert ( self.width > 0 ) and ( self.height > 0 )
		assert ( f )

		self.coordinates = []

		# PBM file goes from the top of the bitmap (y = h-1) to the
		# bottom of the bitmap (y = 0), and from the left of the bitmap
		# (x = 0) to the right of the bitmap (x = w)

		# Each line of the file contains a string of one or more characters
		# from the alphabet { '0', '1', '#', '\n' } where
		#
		#   '0' -- a zero bit in the bitmap
		#   '1' -- a one bit in the bitmap
		#   '#' -- introduces a comment line
		#   '\n' -- a line record terminator
		#
		# Note the last line of the file may possibly omit the trailing LF.
		# That is normal for PBM files of type P1.
		#
		# Each line from the file may be a portion of one or more rows
		# of the bitmap.  So, it's up to use to track which row and column
		# we are at in the bitmap.

		# Our column index
		column = 0

		# Our row index.  Recall that we start at the top row, row h - 1
		row = self.height - 1

		# Now loop over the remaining lines in the file
		# Note that the file line of a P1 PBM file usually does not
		# end with a LF record terminator

		for line in f:

			# Ignore semantically empty lines
			line = line.strip()
			if ( line[0] == '' ) or ( line[0] == '#' ):
				continue

			# Too much data in the file?
			if row <= -1:
				sys.stderr.write( 'Too much data in %s\n' % self.infile )
				return False

			# Loop over each byte in the line
			for i in range( 0, len( line )):

				if line[i] == '1':
					self.coordinates.append(( column, row ))
				elif line[i] != '0':
					sys.stderr.write( 'Invalid content in %s\n' % self.infile )
					return False

				# Move to the next column
				column += 1

				# Have we finished this row?
				if column >= self.width:

					# Finished a row, move down to the next row
					column = 0
					row -= 1

		# All done
		# Perform a sanity check: we should be at the start of row -1
		if ( column == 0 ) and ( row == -1 ):
			return True

		# Something bad happened
		sys.stderr.write( ' Premature end-of-file encountered in %s\n' % self.infile )
		return False

	# Load a file in which each line has the format
	#
	#    x-coord y-coord radius

	def __load_xyr( self, f ):

		assert( f )

		self.coordinates = []
		self.width, self.height = int( self.BOXSIZE ), int( self.BOXSIZE )
		px, py = [], []

		for line in f:

			# Ignore comment lines
			if line.startswith( '#' ):
				continue

			vals = line.strip().split(' ')
			if ( len( vals ) < 2 ) or ( len( vals ) > 3 ):
				sys.stderr.write( 'Invalid content in file %s\n' % self.infile )
				return False

			px.append( float( vals[0] ) )
			py.append( float( vals[1] ) )

		# Find the extrema
		fmin = min( min( px ), min( py ) )
		fmax = max( max( px ), max( py ) )

		# We will translate bounding box containing the points to have
		# it's bottom, left corner at (0, 0).  Further we will scale the
		# box to have height and width of BOXSIZE (e.g., 800).  Then
		# we convert the floating point values to integers

		# Note we pretend the points all have radius zero....

		span = float( fmax - fmin )
		scale = float( self.BOXSIZE / span ) if span != 0 else float( 1 )
		# Can't do "for x, y in px, py:" as the lists are too large
		# resulting in 'too manu values to unpack'
		for i in range( 0, len( px ) ):
			self.coordinates.append( ( int( round( (px[i] - fmin) * scale ) ),
										int( round( (py[i] - fmin) * scale ) ) ) )

		return True

	def load( self, infile ):

		# Deal with a missing .pbm extension
		self.infile = infile
		if not os.path.exists( self.infile ):
			if os.path.exists( self.infile + '.pbm' ):
				self.infile += '.pbm'
			elif os.path.exists( self.infile + '.PBM' ):
				self.infile += '.PBM'
			elif os.path.exists( self.infile + '.pts' ):
				self.infile += '.pts'
			else:
				# Well, we're going to get an error when we try
				# to open that input file....
				pass

		# Open the input file
		# This may raise an exception which is fine by us
		f = open( self.infile, 'rb' )

		# Get the magic number
		# For PBM files this will always be two bytes followed by a \n
		# For other image types, this line could be who knows what.  Hence
		# our use of a size argument to readline()
		magic_number = f.readline(4).strip()

		# PBM files must be P1 or P4
		if magic_number in ['P4', 'P1']:

			# File is a PBM bitmap file

			# Loop until we read the bitmap dimensions
			# NOTE: we cannot use "while line in f:" since that is incompatible
			# with later using f.read().  If the file is of type P4, then we
			# will need to use f.read() to obtain the bitmap

			self.width, self.height = ( 0, 0 )
			while True:
				line = f.readline()
				if not line.startswith( '#' ):
					self.width, self.height = tuple( map( int, line.split() ) )
					break

			# Did we actually read anything (useful)?
			if ( self.width == 0 ) or ( self.height == 0 ):
				sys.stderr.write( 'Unable to read sensible bitmap dimensions for %s\n' % self.infile )
				f.close()
				return False

			# Now read the bitmap
			# cities will be a list of 2-tuples, each 2-tuple being the (x, y)
			# coordinate of a 1 bit in the bitmap.  These (x, y) coordinates
			# correspond to row and column numbers with
			#
			#    0 <= row <= height - 1
			#    0 <= column <= width - 1
			#
			# row = 0 corresponds to the bottom of the bitmap
			# column = 0 corresponds to the left edge of the bitmap

			ok = self.__load_pbm_p4( f ) if magic_number[1] != '1' \
				else self.__load_pbm_p1( f )

		elif magic_number == '# x-':

			# File may be an (x, y, radius) coordinate file
			line = f.readline().strip()
			if line != 'coord y-coord radius':
				sys.stderr.write( 'Input file %s is not a supported file type\n' % self.infile )
				sys.stderr.write( 'Must be a PBM file or file of (x, y) coordinates\n' )
				f.close()
				return False

			ok = self.__load_xyr( f )

		else:

			# Unsupported file type
			sys.stderr.write( 'Input file %s is not a supported file type\n' % self.infile )
			sys.stderr.write( 'Must be a PBM file or file of (x, y) coordinates\n' )
			f.close()
			return False


		# All done with the input file
		f.close()

		# If ok is False, then __load_xxx() will have printed an error
		# message already
		return ok

	def write_tspfile( self, outfile='', f=None, infile='TSPART' ):

		if not f:
			# Deal with funky outfile names
			if ( not outfile ) or ( outfile == '' ):
				if ( self.infile.endswith( '.pbm' ) ):
					outfile = self.infile[:-3] + 'tsp'
				elif ( self.infile.endswith( '.PBM' ) ):
					outfile = self.infile[:-3] + 'TSP'
				else:
					outfile = self.infile + '.tsp'

			# Create the output file
			# This may generate an exception which is fine by us
			f = open( outfile, 'w' )

		# And now write the contents of the TSPLIB file
		try:
			# Header
			f.write( 'NAME:%s\n' % infile )
			f.write( 'TYPE:TSP\n' )
			f.write( 'DIMENSION:%d\n' % len( self.coordinates ) )
			f.write( 'EDGE_WEIGHT_TYPE:EUC_2D\n' )
			f.write( 'NODE_COORD_TYPE:TWOD_COORDS\n' )

			# list of coordinates
			f.write( 'NODE_COORD_SECTION:\n' )
			city_number = 0
			for city in self.coordinates:
				f.write( '%d %d %d\n' % ( city_number, city[0], city[1] ) )
				city_number += 1

			# And finally an EOF record
			f.write( 'EOF:\n' )

		except:
			# Remove the incomplete file
			# Note on Windows we must close the file before deleting it
			f.close()
			if outfile != '':
				os.unlink( outfile )
			# Now re-raise the exception
			raise

		f.close()

	# max_segments == 0 implies unlimited number of segments per path
	def write_tspsvg( self, outfile, tour, max_segments=400,
					line_color='#000000', fill_color='none',
					file_contents='3', label=None ):

		assert( outfile )
		assert( tour )
		assert( int( max_segments ) >= 0 )


		# ms will limit number of points in the path and hence we need
		# ms = max_segments + 1 unless max_segments = 0
		# Note that previously we ensured that max_segments >= 0
		ms = int( max_segments )
		if ms != 0:
			ms += 1

		# Default line color to black
		if ( not line_color ) or ( line_color == '' ):
			line_color = '#000000'

		# Note, we only ask for a fill color when we know we're drawing
		# a single, closed path
		if fill_color:
			fill_color = fill_color.strip( '"\'' )
		if ( not fill_color ) or ( fill_color == '') or ( ms != int( 0 ) ):
			fill_color = 'none'

		f = open( outfile, 'w' )

		tx = ( 3200 - self.width ) / 2 if self.width else 0
		ty = ( 800 + self.height ) / 2 if self.height else 0

		# Write the SVG preamble?
		if ( 1 & int( file_contents ) ):
			f.write(
'<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n' +
'<!-- Created with the Eggbot TSP art toolkit (http://egg-bot.com) -->\n' +
'\n' +
'<svg xmlns="http://www.w3.org/2000/svg"\n' +
'     xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"\n' +
'     xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"\n' +
'     xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"\n' +
'     xmlns:dc="http://purl.org/dc/elements/1.1/"\n' +
'     xmlns:cc="http://creativecommons.org/ns#"\n' +
'     height="800"\n' +
'     width="3200">\n' +
'  <sodipodi:namedview\n' +
'            showgrid="false"\n' +
'            showborder="true"\n' +
'            inkscape:showpageshadow="false"/>\n' +
'  <metadata>\n' +
'    <rdf:RDF>\n' +
'      <cc:Work rdf:about="">\n' +
'        <dc:format>image/svg+xml</dc:format>\n' +
'        <dc:type rdf:resource="http://purl.org/dc/dcmitype/StillImage" />\n' +
'        <dc:subject>\n' +
'          <rdf:Bag>\n' +
'            <rdf:li>Egg-Bot</rdf:li>\n' +
'            <rdf:li>Eggbot</rdf:li>\n' +
'            <rdf:li>TSP</rdf:li>\n' +
'            <rdf:li>TSP art</rdf:li>\n' +
'          </rdf:Bag>\n' +
'        </dc:subject>\n' +
'        <dc:description>TSP art created with the Eggbot TSP art toolkit (http://egg-bot.com)</dc:description>\n' +
'      </cc:Work>\n' +
'    </rdf:RDF>\n' +
'  </metadata>\n')

		# Now open the SVG group
		f.write('  <g ')

		if label and ( label != '' ):
			f.write('inkscape:groupmode="layer" ' + \
				'inkscape:label="%s"\n' % label.replace( '&', '&amp;' ).replace( '"', '&quot;' ) )

		f.write(
'     transform="translate(%d, %d) scale(1, -1)">\n' % ( tx, ty ) )

		max_index = len( self.coordinates )
		last_city = None
		path = False
		first_path = True
		points = 0

		for city_idx in tour:

			city_index = int( city_idx )
			if ( city_index < 0 ) or ( city_index >= max_index ):
				sys.stderr.write( 'TSP tour contains an invalid city index, %s\n' % city_index )
				f.close()
				os.unlink( outfile )
				return False

			if not path:
				# We need to start a new path whose first point is the
				# last city we moved to
				path = True
				if not last_city:
					last_city = self.coordinates[city_index]
				f.write( '    <path style="fill:%s;stroke:%s;stroke-width:1"\n' % ( fill_color, line_color ) +
						'          d="m %d,%d' % last_city )
				if points == 0:
					# This is the first path so skip the next step
					continue

			# Now move to the current city
			next_city = self.coordinates[city_index]
			f.write( ' %d,%d' % ( next_city[0] - last_city[0],
					      next_city[1] - last_city[1] ) )
			last_city = next_city
			points += 1

			if ( ms != 0 ) and ( points > ms ):
				# Start a new path
				path = False
				first_path = False
				points = 1 # 1 and not 0
				f.write( '"/>\n' )

		# Close out any open path
		if path:
			if first_path:
				# Make sure it's known that this is a single, closed path
				# Note: if we wrote a single path but closed it out because
				# ien( tour ) == ms+1, then this final 'Z' will be omitted
				# whish should be okay anyway.
				f.write( ' Z"/>\n' )
			else:
				f.write( '"/>\n' )

		# Close out the SVG document
		f.write( '  </g>\n' )

		# Write the SVG postamble?
		if ( 2 & int( file_contents) ):
			f.write( '</svg>\n' )

		return True

if __name__ == '__main__':

	def fixup_args( argv ):
		if len( argv ) == 0:
			# Prompt for input and output file names
			infile = ''
			outfile = ''
			while infile == '':
				infile = raw_input( 'Input file: ' )
			while outfile == '':
				outfile = raw_input( 'Output file: ' )
			return ( infile, outfile )
		elif len( argv ) == 1:
			# Assume output file name is derived from the input file name
			if argv[0].endswith( '.pbm' ):
				# Output file name is input file name - 'pbm' + 'tsp'
				return ( argv[0], argv[0][:-3] + 'tsp' )
			elif argv[0].endswith( '.PBM' ):
				# Output file name is input file name - 'PBM' + 'TSP'
				return ( argv[0], argv[0][:-3] + 'TSP' )
			elif argv[0].endswith( '.pts' ):
				return ( argv[0], argv[0][:-3] + 'tsp' )
			elif os.path.exists( argv[0] ):
				# Output file name is input file name + '.tsp'
				return ( argv[0], argv[0] + '.tsp' )
			elif os.path.exists( argv[0] + '.pbm' ):
				return ( argv[0] + '.pbm', argv[0] + '.tsp' )
			elif os.path.exists( argv[0] + '.PBM' ):
				return ( argv[0] + '.PBM', argv[0] + '.TSP' )
			elif os.path.exists( argv[0] + '.pts' ):
				return ( argv[0] + '.pts', argv[0] + '.tsp' )
		elif len( argv ) == 2:
			return ( argv[0], argv[1] )
		else:
			return ( '', '' )

	(infile, outfile) = fixup_args( sys.argv[1:] )

	if ( infile == '' ) or ( outfile == '' ):
		sys.stderr.write( 'Usage: %s [input-bitmap-file [output-tsplib-file]]\n' % sys.argv[0] )
		sys.exit( 1 )

	citymap = tspBitCity()
	if not citymap.load( infile ):
		sys.exit( 1 )

	citymap.write_tspfile( outfile )
