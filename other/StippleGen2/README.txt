
 StippleGen_2
 
 SVG Stipple Generator, v. 2.02
 Copyright (C) 2012 by Windell H. Oskay, www.evilmadscientist.com
 
 Full Documentation: http://wiki.evilmadscience.com/StippleGen
 Blog post about the release: http://www.evilmadscientist.com/go/stipple2
 
 
 An implementation of Weighted Voronoi Stippling:
 http://mrl.nyu.edu/~ajsecord/stipples.html
 
 
 *******************************************************************************
 
 Change Log:
 
 v 2.02
 * Force files to end in .svg
 * Fix bug that gave wrong size to stipple files saved white stipples on black background
 
 v 2.01:  
 * Improved handling of Save process, to prevent accidental "not saving" by users.
 
 v 2.0:  
 * Add tone reversal option (white on black / black on white)
 * Reduce vertical extent of GUI, to reduce likelihood of cropping on small screens
 * Speling corections 
 * Fixed a bug that caused unintended cropping of long, wide images
 * Reorganized GUI controls
 * Fail less disgracefully when a bad image type is selected.
 
 ******************************************************************************* 
 
 Program is based on the Toxic Libs Library ( http://toxiclibs.org/ )
 & example code:
 http://forum.processing.org/topic/toxiclib-voronoi-example-sketch
 
 
 Additional inspiration:
 Stipple Cam from Jim Bumgardner
 http://joyofprocessing.com/blog/2011/11/stipple-cam/
 
 and 
 
 MeshLibDemo.pde - Demo of Lee Byron's Mesh library, by 
 Marius Watz - http://workshop.evolutionzone.com/
 
 
 Requires ControlP5 library and Toxic Libs library:
 http://www.sojamo.de/libraries/controlP5/
 http://hg.postspectacular.com/toxiclibs/downloads
 


 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * http://creativecommons.org/licenses/LGPL/2.1/
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA