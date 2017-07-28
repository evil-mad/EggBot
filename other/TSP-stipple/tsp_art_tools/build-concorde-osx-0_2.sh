 #!/bin/sh
#
# 9/27/2010-A
# Download and build the Concorde TSP Solver for OS X
# Tested on OS X 10.6.4 with gcc 4.2.1 from the Apple Xcode distribution
#
# As this script defaults to using /usr/local/src and /usr/local/bin, you
# will want to run this script as root; e.g.,
#
#    % sudo build-concorde.sh
#
# **** NOTE ********
# YOU MUST HAVE gcc, make, curl, and likely some other tools as well
# to run this script.  Curl comes with OS X.  The others you will have
# if you have installed Apple's Xcode Toolkit.  There may be a copy
# on your OS X install DVDs (but you need to hunt around to find the
# package on the DVD).  Or, if you have a (free) developer account with
# Apple, you can log in to the "Mac Dev Center" to download the toolkit.
# See http://developer.apple.com/technologies/xcode.html.
# ******************
#
# Daniel C. Newman, 21 September 2010
# dan dot newman at mtbaldy dot us
#
# Where to store the source tree
#
SRCDIR=/usr/local/src/concorde
#
# Where to make the binaries visible from
# (Concorde's makefile doesn't have an "install" target)
#
BINDIR=/usr/local/bin
#
# What to build (32 bit and/or 64 bit)
#BUILD32=0
BUILD64=1
#
# Because we like to watch the commands go by....
set -x
#
if [ $BUILD32 -eq 0 -a $BUILD64 -eq 0 ]; then
    echo "Looks like there's nothing to do; goodbye"
    exit 0
fi
#
if [ $BUILD32 -ne 0 ]; then
    # 32bit libraries
    mkdir -p $SRCDIR/qsopt32
    cd $SRCDIR/qsopt32
    curl -O http://www2.isye.gatech.edu/~wcook/qsopt/beta/codes/mac/qsopt.a
    curl -O http://www2.isye.gatech.edu/~wcook/qsopt/beta/codes/mac/qsopt.h
    # You don't actually need the qsopt executable
    curl -O http://www2.isye.gatech.edu/~wcook/qsopt/beta/codes/mac/qsopt
    chmod a+x qsopt
fi
#
if [ $BUILD64 -ne 0 ]; then
    # 64bit libraries
    mkdir -p $SRCDIR/qsopt64
    cd $SRCDIR/qsopt64
    curl -O http://www.math.uwaterloo.ca/~bico/qsopt/beta/codes/mac64/qsopt.a
    curl -O http://www.math.uwaterloo.ca/~bico/qsopt/beta/codes/mac64/qsopt.h
    # You don't actually need the qsopt executable
    curl -O http://www2.isye.gatech.edu/~wcook/qsopt/beta/codes/mac64/qsopt
    chmod a+x qsopt
fi
#
# Now download the Concorde TSP solver package and buld it
#
cd $SRCDIR/..
curl -O http://www.math.uwaterloo.ca/tsp/concorde/downloads/codes/src/co031219.tgz
# will unpack to ./concorde
tar -xvf co031219.tgz
cd concorde
#
if [ $BUILD32 -ne 0 ]; then
    #
    # Build the 32bit version of Concorde
    BUILDDIR=osx-build-32
    #
    # MUST USE AN ABSOLUTE DIRECTORY PATH FOR LOCATION OF qsopt LIBRARIES !!!
    QSOPTDIR=`pwd`/qsopt32
    mkdir $BUILDDIR
    cd $BUILDDIR
    # Ignore the "checking host system type" warning ...
    # You must provide the --host switch but it doesn't have to be correct
    CFLAGS="-g -O3 -m32" ../configure --with-qsopt=$QSOPTDIR --host=darwin
    make
fi
#
if [ $BUILD64 -ne 0 ]; then
    #
    # Build the 64bit version of Concorde
    BUILDDIR=osx-build-64
    #
    # Again, you must use an absolute directory path
    cd $SRCDIR
    QSOPTDIR=`pwd`/qsopt64
    mkdir $BUILDDIR
    cd $BUILDDIR
    # Again, ignore the checking host system type warning ...
    CFLAGS="-g -O3 -m64" ../configure --with-qsopt=$QSOPTDIR --host=darwin
    make
fi
#
# Note: if both 32 and 64bit builds were done, then the last of the
# two built is the one we set up these symbolic links for.  That's
# because whichever last set BUILDDIR "wins"
#
if [ -x $SRCDIR/$BUILDDIR/TSP/concorde ] ; then
   if [ -L $BINDIR/concorde ] ; then
       unlink $BINDIR/concorde
   fi
   if [ -x $BINDIR/concorde ] ; then
       echo "$BINDIR/concorde already exists"
   else
       ln -s $SRCDIR/$BUILDDIR/TSP/concorde $BINDIR/concorde
   fi
fi
if [ -x $SRCDIR/$BUILDDIR/LINKERN/linkern ] ; then
   if [ -L $BINDIR/linkern ] ; then
       unlink $BINDIR/linkern
   fi
   if [ -x $BINDIR/linkern ] ; then
       echo "$BINDIR/linkern already exists"
   else
       ln -s $SRCDIR/$BUILDDIR/LINKERN/linkern $BINDIR/linkern
   fi
fi
