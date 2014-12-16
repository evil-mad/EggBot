# tspart.py
# 3/7/2012
#
# Interpret the black bits in a black & white bit map as "cities" on a
# rectangular map.  Use the coordinates of these cities (bits) as input
# to a TSP solver by converting the coordinates of the cities to a TSPLIB
# file.
#
# Then, generate a fast, approximate solution to the resulting TSP using
# the linkern solver from Concorde TSP.  Using the solution from the
# solver -- a "tour" -- generate an SVG plot of the tour.
#
#    python tspart.py [input-bitmap-file [output-svg-file]]
#
# If no input file name is supplied, they you will be prompted for the
# name of an input and output file.  If no output file name is supplied,
# then it will have a name similar to the input file but with a ".svg"
# extension.
#
# Presently, the input file formats supported are
#
# .PBM -- Portable Bit Map files (Raw or ASCII; P4 or P1)
#
# .PTS -- File of (x, y) or (x, y, radius) coordinates.  Must have as the
#         first line the literal string
#
#            # x-coord y-coord radius
#
#         Subsequent lines must then be either
#
#            x-coordinate y-coordinate radius
#
#         or
#
#            x-coordinate y-coordinate
#
#         where "x-coordinate", "y-coordinate", and "radius" are the ASCII
#         representation of floating point numbers.  E.g.,
#
#            # x-coord y-coord radius
#            0.0210369 0.00199109 0.0022353
#            0.0255807 0.00200347 0.00216036
#            0.115518 0.00203477 0.00263275
#
#         The radii are ignored.


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
import getopt
import tempfile
import subprocess
from tspbitcity import *
from tspsolution import *

# Path to the linkern executable
if sys.platform.lower() == 'win32':
	LINKERN = 'C:\linkern.exe'
	use_shell = False
else:
	LINKERN = '/usr/local/bin/linkern'
	use_shell = True

# linkern switches
LINKERN_OPTS = ' -r %s -o %s %s'

# Number of linkern runs to take
linkern_runs = 3

# Simply report the number of stipples?
stipple_report_only = False

# Maximum number of line segments per <path>
max_segments = int( 400 )

# Fill color for closed paths
fill_color = 'none'

# Stroke color for lines
line_color = '#000000'

# Complete svg file (3); just the start (--pre; 1); just the end (--post; 2)
file_contents = 3

# Name for this SVG layer
layer_name = None

# Output our usage and then exit
# When exit_stat is non-zero, write to stderr; otherwise, write to stdout

def usage( prog, exit_stat=0 ):
	str = \
'Usage: %s [-ch] [-r n] [-s exe] [input-bitmap-file [output-svg-file]]\n' % prog
	str += \
' -c, --count\n' + \
'    Report the number of stipples in the input file and then exit\n' + \
' -f color, --fill=color\n' + \
'    Fill color (e.g., red, blue, #ff0000); requires --max-segments=0 (default=%s)\n' % fill_color
	str += \
' -h, --help\n' + \
'    This message\n' + \
' -L name, --layer=name\n' + \
'    Layer name (default=None)\n' + \
' -m n, --max-segments=n\n' + \
'    Maximum number of line segments per SVG <path> element (default --max-segments=%s)\n' % max_segments
	str += \
' --mid, --pre, --post\n' + \
'    Produce output with only the SVG preamble (--pre), postamble (--post), or neither (--mid)\n' + \
' -r n, --runs=n\n' + \
'    Number of linkern runs to take (default --runs=%s)\n' % linkern_runs
	str += \
' -s color, --stroke=color\n' + \
'    Stroke (line) color (e.g., black, green, #000000; default=%s)\n' % line_color
	str += \
' -S exe-path, --solver=exe-path\n' + \
'     File path for the linkern executable (default --solver=%s)\n' % LINKERN

	if exit_stat:
		sys.stderr.write( str )
	else:
		sys.stdout.write( str )
	sys.exit( exit_stat )
# Determine the names of our input and output files
# Input bitmap file name
# Output SVG file name
infile = ''
outfile = ''

try:
	opts, args = getopt.getopt( sys.argv[1:], 'cf:hl:L:m:r:s:S:',
		[ 'count', 'fill=', 'help', 'layer=', 'line-color=',
		  'max-segments=', 'mid', 'post', 'pre', 'runs=',
		  'stroke=', 'solver=' ] )
except:
	usage( sys.argv[0], 1 )

for opt, val in opts:
	if opt in ( '-c', '--count' ):
		stipple_report_only = True
	elif opt in ( '-f', '--fill' ):
		fill_color = val
	elif opt in ( '-h', '--help' ):
		usage( sys.argv[0], 0 )
	elif opt in ( '-s', '--stroke', '--line-color' ):
		line_color = val
	elif opt in ( '-L', '--layer' ):
		layer_name = val.strip( '"\'' )
	elif opt in ( '-m', '--max-segments' ):
		if int( val ) >= 0:
			max_segments = int( val )
	elif opt in ( '--mid' ):
		file_contents = 0
	elif opt in ( '--post' ):
		file_contents = 2
	elif opt in ( '--pre' ):
		file_contents = 1
	elif opt in ( '-r', '--runs' ):
		linkern_runs = val if int( val ) > 0 else '1'
	elif opt in ( '-S', '--solver' ):
		LINKERN = val

# Enforce -max-segments=0 when --fill is used
if ( int( max_segments ) != 0 ) and ( fill_color != 'none' ):
	sys.stderr.write( 'Use of -f or --fill requires -max-segments=0\n' )
	usage( sys.argv[0], 1 )

# Look to our command line arguments for possible input/output file names
if len( args ) == 0:
	while infile == '':
		try:
			infile = raw_input( 'Input file: ' )
		except:
			print
			sys.exit( 0 )
	while outfile == '':
		try:
			outfile = raw_input( 'Output file: ' )
		except:
			print
			sys.exit( 0 )
elif len( args ) == 1:
	infile = args[0]
elif len( args ) == 2:
	infile = args[0]
	outfile = args[1]
else:
	os.stderr.write( 'Usage: %s [input-bitmap-file [output-svg-file]]\n' % sys.argv[0] )
	os.sysexit(1)


# Now do some fixups, including defaulting the output file name
if infile.endswith( '.pbm' ) or infile.endswith( '.pts' ):
	tmp_prefix = infile[:-3]
	solfile = infile[:-3] + 'tour'
	if outfile == '':
		outfile = infile[:-3] + 'svg'
elif infile.endswith( '.PBM' ) or infile.endswith( '.PTS' ):
	tmp_prefix = infile[:-3]
	solfile = infile[:-3] + 'TOUR'
	if outfile == '':
		outfile = infile[:-3] + 'SVG'
else:
	tmp_prefix = os.path.split( infile )[1]
	solfile = infile + '.tour'
	if outfile == '':
		outfile = infile + '.svg'

# Place the solution file into the temporary directory.  We don't need to
# worrry (too much) about creating it: we're going to make some other calls
# to open a temporary file and those calls should instantiate the directory.
# And, since we check for errors on those calls, we should catch any problems.

solfile = os.path.join(tempfile.gettempdir(), os.path.basename(solfile))

# Load the bitmap file
print 'Loading bitmap file %s ... ' % infile,
cities = tspBitCity()
if not cities.load( infile ):
	sys.exit(1)
print 'done; %d stipples' % len( cities.coordinates )
if stipple_report_only:
	sys.exit( 0 )

# Open a temporary file to hold the TSPLIB file
tsp_fd, tspfile = tempfile.mkstemp( suffix='.tsp', prefix=tmp_prefix, text=True )
if tsp_fd < 0:
	sys.stderr.write( 'Unable to open a temporary file\n' )
	sys.exit(1)

# Convert this file descriptor to a Python file object
tsp_f = os.fdopen(tsp_fd, 'w')

# Now write the TSPLIB file
print 'Writing TSP solver input file %s ... ' % tspfile,
cities.write_tspfile( tspfile, tsp_f )
print 'done'

# Run the solver
print 'Running TSP solver ... '
cmd = LINKERN + LINKERN_OPTS % ( linkern_runs, solfile, tspfile )
pipe = subprocess.Popen( cmd, shell=use_shell )
status = pipe.wait()

# Remove the temporary TSPLIB file
os.unlink(tspfile)

# Did the solver succeed?
if status:
	# No, something went wrong
	sys.stderr.write( 'Solver failed; status = %s\n' % status )
	os.unlink( solfile )
	os.sysexit( 1 )

# Solver succeeded
print '\nSolver finished successfully'

# Load the solution (a tour)
print 'Loading solver results from %s ... ' % solfile,
solution = tspSolution()
if not solution.load( solfile ):
	sys.stderr.write( 'Unable to load the solution file\n' )
	os.unlink( solfile )
	os.sysexit( 1 )
print 'done'

# Remove the tour file
os.unlink(solfile)

# Now write the SVG file
print 'Writing SVG file %s ... ' % outfile,
if not cities.write_tspsvg( outfile, solution.tour, max_segments,
							line_color, fill_color, file_contents,
							layer_name ):
	# write_tspsvg() takes care of removing outfile
	sys.stderr.write( 'Error writing SVG file\n' )
	os.sysexit( 1 )
print 'done'
