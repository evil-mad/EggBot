# Generate Eggbot-friendly SVG drawings from "point" output files
# produced by Adrian Secord's weighted Voronoi stippler.
#
# The data from the file consists of the (x, y) coordinate and radius r
# of each stipple.  This data is read and parsed, crudely sorted
# so as to produce left-to-right and then right-to-left pen arm travel
# (instead of left-to-right, return, left-to-right), and then rescaled
# to fill the SVG document while preserving an x:y aspect ratio of 1:1.

# Written by Daniel C. Newman
# dan dot newman at mtbaldy dot us
# January 2011

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import os
import sys

class Stipples:

	# Output SVG document dimensions

	WIDTH = 3200
	HEIGHT = 800

	# When we draw a circle in SVG, the stroke will be both inside and
	# outside of the circle.  We therefore reduce the circle's radius
	# by OUTER_RADIUS_REDUCE pixels so that the circle drawn by the pen
	# lies within the actual stipple.  This value of 2 pixels is
	# likely too small: adjust as you see fit.
	#
	# Also, any stipple whose radius is less than OUTER_RADIUS_REDUCE
	# will be drawn as a point and not a circle.

	OUTER_RADIUS_REDUCE = float( 2.0 )

	def __init__( self ):

		# We save the input bitmap file name for purposes of
		# error reporting and generating a default output file name
		self.infile = ''

		# Our list of (x, y, r) coordinates
		self.coordinates = []

	def load_xyr( self, f ):

		# Load a file in which each line has the format
		#
		#    x-coord y-coord radius

		assert( f )

		self.coordinates = []
		px, py, pr = [], [], []

		# Track bounding box information
		# We take the radius of the stipples into account

		xmin, xmax = float( 'inf' ), float( '-inf' )
		ymin, ymax = float( 'inf' ), float( '-inf' )

		for line in f:

			# Ignore comment lines
			if line.startswith( '#' ):
				continue

			vals = line.strip().split(' ')
			if ( len( vals ) < 2 ) or ( len( vals ) > 3 ):
				sys.stderr.write( 'Invalid content in file %s\n' % self.infile )
				return False

			x = float( vals[0] )
			y = float( vals[1] )
			r = float( vals[2] )
			if r < 0.0:
				r = float( 0.0 )
			if ( x - r ) < xmin:
				xmin = x - r
			elif ( x + r ) > xmax:
				xmax = x + r
			if ( y - r ) < ymin:
				ymin = y - r
			elif ( y + r ) > ymax:
				ymax = y + r

			px.append( x )
			py.append( y )
			pr.append( r )

		# Find the extrema
		dmin = xmin if xmin < ymin else ymin
		dmax = xmax if xmax > ymax else ymax

		# Width of the bounding box
		span = dmax - dmin

		# Scale the box to be BOXSIZE high and wide
		width = self.WIDTH if self.WIDTH <= self.HEIGHT else self.HEIGHT
		scale = float( width / span ) if span != 0.0 else float( 1.0 )

		iy = 0.0
		flip = True  # back and forth flip/flop
		lastx = px[0] - 0.005
		for i in range( 0, len( px ) ):
			if lastx > px[i]:
				flip = not flip
				iy += 2.0
			if flip:
				vsort = iy + px[i] - xmin
			else:
				vsort = iy + xmax - px[i]
			lastx = px[i]
			self.coordinates.append( [ vsort,
						int( round( (px[i] - dmin) * scale ) ),
						int( round( (ymax - ( py[i] - dmin) ) * scale ) ),
						scale * pr[i] ] )

		# Sort the coordinates in a fashion to minimize back and
		# forth pen motion
		self.coordinates.sort()
		return True

	def load( self, infile ):

		# Deal with a missing .pts extension
		self.infile = infile
		if not os.path.exists( self.infile ):
			if os.path.exists( self.infile + '.pts' ):
				self.infile += '.pts'

		# Open the input file
		# This may raise an exception which is fine by us
		f = open( self.infile, 'rb' )

		# Check the first few bytes of the first line of the file
		magic_number = f.readline(4).strip()

		# Note: I took this code here from a larger program which
		# supported several input formats including some which used
		# Unix-style "magic numbers" at the start of the file
		if magic_number == '# x-':

			# File may be an (x, y, radius) coordinate file
			line = f.readline().strip()
			if line != 'coord y-coord radius':
				sys.stderr.write( 'Input file %s is not a supported file type\n' % self.infile )
				sys.stderr.write( 'Must be a file of (x, y, r) coordinates\n' )
				f.close()
				return False

			ok = self.load_xyr( f )

		else:

			# Unsupported file type
			sys.stderr.write( 'Input file %s is not a supported file type\n' % self.infile )
			sys.stderr.write( 'Must be a PBM file of (x, y, r) coordinates\n' )
			f.close()
			return False

		# All done with the input file
		f.close()

		# If ok is False, then load_xyr() will have printed an error
		# message already
		return ok

	def write_svgfile( self, outfile='', f=None, infile='TSPART' ):

		if f is None:
			if ( not outfile ) or ( outfile == '' ):
				if ( self.infile.endswith( '.pts' ) ):
					outfile = self.infile[:-3] + 'svg'
				elif ( self.infile.endswith( '.PTS' ) ):
					outfile = self.infile[:-3] + 'SVG'
				else:
					outfile = self.infile + '.svg'

			# Create the output file
			# This may generate an exception which is fine by us
			f = open( outfile, 'w' )

		# And now write the contents of the SVG file
		try:
			f.write(
'<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n' +
'<svg xmlns="http://www.w3.org/2000/svg"\n' +
'     xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"\n' +
'     xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"\n' +
'     xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"\n' +
'     xmlns:dc="http://purl.org/dc/elements/1.1/"\n' +
'     xmlns:cc="http://creativecommons.org/ns#"\n' +
'     width="%d"\n' % self.WIDTH +
'     height="%d">\n' % self.HEIGHT )
			f.write(
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
'            <rdf:li>egg-bot</rdf:li>\n' +
'            <rdf:li>eggbot</rdf:li>\n' +
'          </rdf:Bag>\n' +
'        </dc:subject>\n' +
'        <dc:description>Stippled artwork intended for plotting on eggs with an Eggbot (http://www.egg-bot.com)</dc:description>\n' +
'      </cc:Work>\n' +
'    </rdf:RDF>\n' +
'  </metadata>\n' +
'  <g style="fill:none;stroke:#000000;stroke-width:1">\n')

			for P in self.coordinates:
				# P = [ sort-value, x, y, radius ]
				x = P[1]
				y = P[2]
				# Remove 2 pixels from the radius to account
				# for the fact that Inkscape will stroke the
				# line with half the width outside the circle
				# AND to (somewhat) account for the actual pen
				# tip width
				r = P[3] - self.OUTER_RADIUS_REDUCE
				path = ''
				if r < self.OUTER_RADIUS_REDUCE:
					# Just draw a point
					# path = 'M %f,%f l 0.001,0.001' % ( x, y )
					r = self.OUTER_RADIUS_REDUCE
				# Draw concentric circles using SVG arcs
				while r >= self.OUTER_RADIUS_REDUCE:
					x1 = x - r
					x2 = x + r
					path += ' L %f,%f A %f,%f 0 1 0 %f,%f A %f,%f 0 1 0 %f,%f' % \
						( x1, y, r, r, x2, y, r, r, x1, y )
					r -= 2.0 * self.OUTER_RADIUS_REDUCE

				f.write( '    <path d="M %s"/>\n' % path[2:] )

			f.write( '  </g>\n</svg>\n' )

		except:
			# Remove the incomplete file
			# Note on Windows we must close the file before deleting it
			f.close()
			if outfile != '':
				os.unlink( outfile )
			# Now re-raise the exception
			raise

		f.close()

if __name__ == '__main__':

	def fixup_args( argv ):
		if len( argv ) == 0:
			# Prompt for input and output file names
			infile = ''
			outfile = ''
			while infile == '':
				infile = raw_input( 'Input file name: ' )
			while outfile == '':
				outfile = raw_input( 'Output file name: ' )
			return ( infile, outfile )
		elif len( argv ) == 1:
			# Assume output file name is derived from the input file name
			if argv[0].endswith( '.pts' ):
				# Output file name is input file name - 'pts' + 'svg'
				return ( argv[0], argv[0][:-3] + 'svg' )
			elif argv[0].endswith( '.PTS' ):
				# Output file name is input file name - 'PTS' + 'SVG'
				return ( argv[0], argv[0][:-3] + 'SVG' )
			elif os.path.exists( argv[0] + '.pts' ):
				return ( argv[0] + '.pts', argv[0] + '.svg' )
			elif argv[0].rfind('.') >= 0:
				return ( argv[0], argv[0][:argv[0].rfind('.')] + '.svg' )
			else:
				return ( argv[0], argv[0] + '.svg' )
		elif len( argv ) == 2:
			return ( argv[0], argv[1] )
		else:
			return ( '', '' )

	(infile, outfile) = fixup_args( sys.argv[1:] )

	if ( infile == '' ) or ( outfile == '' ):
		sys.stderr.write( 'Usage: %s [input-point-file [output-svg-file]]\n' % sys.argv[0] )
		sys.exit( 1 )

	stipples = Stipples()
	if not stipples.load( infile ):
		sys.exit( 1 )

	stipples.write_svgfile( outfile )
