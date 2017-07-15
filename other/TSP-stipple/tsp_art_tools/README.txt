5 October 2010

This directory contains some tools for generating TSP art for the Eggbot.
These tools have only been tested quite a bit under OS X 10.6.
These tools have had very limited testing on Windows XP SP3.

 25 Sep 2010 v0.1   Initial version
 27 Sep 2010 v0.1   Added --count and --runs options
 28 Sep 2010 v0.3   Added support for (x,y) and (x,y,radius) files
  1 Oct 2010 v0.4   Decided to close the TSP tour after all; added
                      --max-segments, and --fill options
  3 Oct 2010 v0.5   Added --line-color, --pre, --mid, & --post options for
                      color TSP art
  4 Oct 2010 v0.6   Added --layer for specifying layer names
  5 Oct 2010 v0.7   Fix help documentation, change --line-color to --stroke
  7 Oct 2010 v0.8   Added cmyk.jsx for Photoshop
 10 Oct 2010 v0.8.1 Added metadata; XML comment referencing Eggbot
  1 Dec 2011 V0.8.2 Fixed an incorrect error diagnostic which when output
                      prematurely ended processing; switched to a different
                      Python library for running linkern in a "subprocess"
                      so as to support current versions of Python on Windows 7
                      (thanks to Peter Vancorenland for diagnosing the issue,
                      researching a fix, and testing it!)
  7 Mar 2012 V0.8.3 Build the tour file in the user's temporary file directory.
                      This avoids issues with file permissions on some systems
                      where it's most convenient to run tspart.py from within
                      a specific directory that the user may not have write
                      access to.  Note that other temporary files were already
                      being written (and then removed) from the temp. directory.
                      For purposes of debugging, the tour file was being
                      written to the current working directory.  That is no
                      longer done.
 15 Jul 2017        The build script is out of date and broken.
                       Current downloads are available at:
                       http://www.math.uwaterloo.ca/~bico/qsopt/downloads/downloads.htm
                       However, it is not clear that 64-bit compatible versions
                       are still available.

build-concorde-osx.sh
  Shell script to obtain with Curl the QSopt LP Solver library and the
  Concorde TSP library and places them in /usr/local/src/concorde.
  The script then builds either a 32bit or 64bit version of the Concorde
  TSP library and places symlinks in /usr/local/bin/ for the concorde
  and linkern executables.  (Note: QSopt is not needed for linkern.)

  This script will need modification for Linux.

  While binary executables are available for concorde and linkern for
  Windows, they require a minimal cygwin install.  See
  http://www.math.uwaterloo.ca/~bico/qsopt/downloads/downloads.htm

cmyk.jsx
  Photoshop script for producing CMYK "separations" for Color TSP art.
  Tested on Mac OS X 10.6.4 with Photoshop CS5.

STIPPLING.txt
  Fifty words or less explaining how to do quick stippling in gimp.  A
  useful means of getting a bitmap to feed to tspart.py.

tspart.py
  Python script to accept as input a black and white bitmap file in PBM
  format and produce as output a SVG file (TSP art) arrived at by using
  the fast, heuristic TSP solver linkern.

  In addition to PBM files, a simple format allowing (x, y) or (x, y, radius)
  coordinates is also supported.  (E.g., the format output by some stippling
  software.) See the comments in tspbitcity.py for further details.

tspbitcity.py
  Python class used by tspart.py.  This is the class which reads in a
  PBM file and can generate a TSPLIB format file for concorde and linkern.
  Also, using a TSP tour, it can generate an SVG file.

  If run as a standalone Python script, tspbitcity.py will generate a
  TSPLIB file from a PBM file.

tspsolution.py
  Python class used by tspart.py.  This class reads a solution file from
  either concorde or linkern and determines the "tour".  This "tour" is
  then used by tspart.py to generate the output SVG file.

-- finit --
