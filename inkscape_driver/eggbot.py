# eggbot.py
# Part of the Eggbot driver for Inkscape
# https://github.com/evil-mad/EggBot
#
# Version 2.8.1, dated June 7, 2017.
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

# TODO: Add and honor advisory locking around device open/close for non Win32

from simpletransform import *

import gettext
import inkex
import simplepath
import string
import time
import ebb_serial		# https://github.com/evil-mad/plotink
import plot_utils		# https://github.com/evil-mad/plotink
import ebb_motion		# https://github.com/evil-mad/plotink	Requires version 0.2 or newer.

import eggbot_conf		#Some settings can be changed here.

F_DEFAULT_SPEED = 1
N_PEN_DOWN_DELAY = 400	# delay (ms) for the pen to go down before the next move
N_PEN_UP_DELAY = 400	# delay (ms) for the pen to up down before the next move

N_PEN_UP_POS = 50		# Default pen-up position
N_PEN_DOWN_POS = 40		# Default pen-down position
N_SERVOSPEED = 50		# Default pen-lift speed
N_WALK_DEFAULT = 10		# Default steps for walking stepper motors
N_DEFAULT_LAYER = 1		# Default inkscape layer

class EggBot( inkex.Effect ):

	def __init__( self ):
		inkex.Effect.__init__( self )

		self.OptionParser.add_option( "--smoothness",
			action="store", type="float",
			dest="smoothness", default=.2,
			help="Smoothness of curves" )
		self.OptionParser.add_option( "--startCentered",
			action="store", type="inkbool",
			dest="startCentered", default=True,
			help="Start plot with pen centered in the y-axis." )
		self.OptionParser.add_option( "--returnToHome",
			action="store", type="inkbool",
			dest="returnToHome", default=True,
			help="Return to home at end of plot." )
		self.OptionParser.add_option( "--wraparound",
			action="store", type="inkbool",
			dest="wraparound", default=True,
			help="Egg (x) axis wraps around-- take shortcuts!" )
		self.OptionParser.add_option( "--penUpSpeed",
			action="store", type="int",
			dest="penUpSpeed", default=F_DEFAULT_SPEED,
			help="Speed (step/sec) while pen is up." )
		self.OptionParser.add_option( "--penDownSpeed",
			action="store", type="int",
			dest="penDownSpeed", default=F_DEFAULT_SPEED,
			help="Speed (step/sec) while pen is down." )
		self.OptionParser.add_option( "--penDownDelay",
			action="store", type="int",
			dest="penDownDelay", default=N_PEN_DOWN_DELAY,
			help="Delay after pen down (msec)." )
		self.OptionParser.add_option( "--penUpDelay",
			action="store", type="int",
			dest="penUpDelay", default=N_PEN_UP_DELAY,
			help="Delay after pen up (msec)." )
		self.OptionParser.add_option( "--engraving",
			action="store", type="inkbool",
			dest="engraving", default=False,
			help="Enable optional engraving tool." )
		self.OptionParser.add_option( "--tab",
			action="store", type="string",
			dest="tab", default="controls",
			help="The active tab when Apply was pressed" )
		self.OptionParser.add_option( "--penUpPosition",
			action="store", type="int",
			dest="penUpPosition", default=N_PEN_UP_POS,
			help="Position of pen when lifted" )
		self.OptionParser.add_option( "--ServoDownSpeed",
			action="store", type="int",
			dest="ServoDownSpeed", default=N_SERVOSPEED,
			help="Rate of lowering pen " )
		self.OptionParser.add_option( "--ServoUpSpeed",
			action="store", type="int",
			dest="ServoUpSpeed", default=N_SERVOSPEED,
			help="Rate of lifting pen " )
		self.OptionParser.add_option( "--penDownPosition",
			action="store", type="int",
			dest="penDownPosition", default=N_PEN_DOWN_POS,
			help="Position of pen when lowered" )
		self.OptionParser.add_option( "--layernumber",
			action="store", type="int",
			dest="layernumber", default=N_DEFAULT_LAYER,
			help="Selected layer for multilayer plotting" )
		self.OptionParser.add_option( "--setupType",
			action="store", type="string",
			dest="setupType", default="controls",
			help="The active option when Apply was pressed" )
		self.OptionParser.add_option( "--manualType",
			action="store", type="string",
			dest="manualType", default="controls",
			help="The active option when Apply was pressed" )
		self.OptionParser.add_option( "--WalkDistance",
			action="store", type="int",
			dest="WalkDistance", default=N_WALK_DEFAULT,
			help="Selected layer for multilayer plotting" )
		self.OptionParser.add_option( "--cancelOnly",
			action="store", type="inkbool",
			dest="cancelOnly", default=False,
			help="Cancel plot and return home only." )
		self.OptionParser.add_option( "--revPenMotor",
			action="store", type="inkbool",
			dest="revPenMotor", default=False,
			help="Reverse motion of pen motor." )
		self.OptionParser.add_option( "--revEggMotor",
			action="store", type="inkbool",
			dest="revEggMotor", default=False,
			help="Reverse motion of egg motor." )

		self.bPenIsUp = None  #Initial state of pen is neither up nor down, but _unknown_.
		self.virtualPenIsUp = False  #Keeps track of pen postion when stepping through plot before resuming
		self.engraverIsOn = False
		self.penDownActivatesEngraver = False
		self.fX = None
		self.fY = None
		self.fPrevX = None
		self.fPrevY = None
		self.ptFirst = None
		self.bStopped = False
		self.fSpeed = 1
		self.resumeMode = False
		self.nodeCount = int( 0 )		#NOTE: python uses 32-bit ints.
		self.nodeTarget = int( 0 )
		self.pathcount = int( 0 )
		self.LayersPlotted = 0
		self.svgLayer = int( 0 )
		self.svgNodeCount = int( 0 )
		self.svgDataRead = False
		self.svgLastPath = int( 0 )
		self.svgLastPathNC = int( 0 )
		self.svgTotalDeltaX = int( 0 )
		self.svgTotalDeltaY = int( 0 )
		self.serialPort = None
		
		nDeltaX = 0
		nDeltaY = 0

		self.svgWidth = float( eggbot_conf.N_PAGE_WIDTH )
		self.svgHeight = float( eggbot_conf.N_PAGE_HEIGHT )
		self.svgTransform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]

		# So that we only generate a warning once for each
		# unsupported SVG element, we use a dictionary to track
		# which elements have received a warning
		self.warnings = {}

		# "Normal" value: self.step_scaling_factor = 2, for 3200 steps/revolution.

		self.step_scaling_factor = eggbot_conf.STEP_SCALE 
		
		self.wrapSteps = 6400 / self.step_scaling_factor
		self.halfWrapSteps = self.wrapSteps / 2
		
	def effect( self ):
		'''Main entry point: check to see which tab is selected, and act accordingly.'''

		self.svg = self.document.getroot()
		self.CheckSVGforEggbotData()

		if ((self.options.tab == '"Help"') or (self.options.tab == '"options"')  or (self.options.tab == '"timing"')):
			pass
		else:
			self.serialPort = ebb_serial.openPort()
			if self.serialPort is None:
				inkex.errormsg( gettext.gettext( "Failed to connect to EggBot. :(" ) )

			if self.options.tab == '"splash"':
				self.allLayers = True
				self.plotCurrentLayer = True
				self.svgNodeCount = 0
				self.svgLastPath = 0
				unused_button = ebb_motion.QueryPRGButton(self.serialPort)	#Query if button pressed
				self.svgLayer = 12345;  # indicate that we are plotting all layers.
				self.plotToEggBot()
	
			elif self.options.tab == '"resume"':
				unused_button = ebb_motion.QueryPRGButton(self.serialPort)	#Query if button pressed
				self.resumePlotSetup()
				if self.resumeMode:
					self.plotToEggBot()
				elif ( self.options.cancelOnly ):
					pass
				else:
					inkex.errormsg( gettext.gettext( "Truly sorry, there does not seem to be any in-progress plot to resume." ) )
	
			elif self.options.tab == '"layers"':
				self.allLayers = False
				self.plotCurrentLayer = False
				self.LayersPlotted = 0
				self.svgLastPath = 0
				unused_button = ebb_motion.QueryPRGButton(self.serialPort)	#Query if button pressed
				self.svgNodeCount = 0;
				self.svgLayer = self.options.layernumber
				self.plotToEggBot()
				if ( self.LayersPlotted == 0 ):
					inkex.errormsg( gettext.gettext( "Truly sorry, but I did not find any numbered layers to plot." ) )
					
			elif self.options.tab == '"setup"':
				self.setupCommand()	

			elif self.options.tab == '"manual"':
				if self.options.manualType == "strip-data":
					for node in self.svg.xpath( '//svg:WCB', namespaces=inkex.NSS ):
						self.svg.remove( node )
					for node in self.svg.xpath( '//svg:eggbot', namespaces=inkex.NSS ):
						self.svg.remove( node )
					inkex.errormsg( gettext.gettext( "I've removed all EggBot data from this SVG file. Have a great day!" ) )
					return	
				else:
					self.manualCommand()

			if self.serialPort is not None:
				ebb_motion.doTimedPause(self.serialPort, 10) #Pause a moment for underway commands to finish...
				ebb_serial.closePort(self.serialPort)	
				
		self.svgDataRead = False
		self.UpdateSVGEggbotData( self.svg )
		return

	def CheckSVGforEggbotData( self ):
		self.svgDataRead = False
		self.recursiveEggbotDataScan( self.svg )
		if ( not self.svgDataRead ):    #if there is no eggbot data, add some:
			eggbotlayer = inkex.etree.SubElement( self.svg, 'eggbot' )
			eggbotlayer.set( 'layer', str( 0 ) )
			eggbotlayer.set( 'node', str( 0 ) )
			eggbotlayer.set( 'lastpath', str( 0 ) )
			eggbotlayer.set( 'lastpathnc', str( 0 ) )
			eggbotlayer.set( 'totaldeltax', str( 0 ) )
			eggbotlayer.set( 'totaldeltay', str( 0 ) )

	def recursiveEggbotDataScan( self, aNodeList ):
		if ( not self.svgDataRead ):
			for node in aNodeList:
				if node.tag == 'svg':
					self.recursiveEggbotDataScan( node )
				elif node.tag == inkex.addNS( 'eggbot', 'svg' ) or node.tag == 'eggbot':
					self.svgLayer = int( node.get( 'layer' ) )
					self.svgNodeCount = int( node.get( 'node' ) )

					try:
						self.svgLastPath = int( node.get( 'lastpath' ) )
						self.svgLastPathNC = int( node.get( 'lastpathnc' ) )
						self.svgTotalDeltaX = int( node.get( 'totaldeltax' ) )
						self.svgTotalDeltaY = int( node.get( 'totaldeltay' ) )
						self.svgDataRead = True
					except:
						node.set( 'lastpath', str( 0 ) )
						node.set( 'lastpathnc', str( 0 ) )
						node.set( 'totaldeltax', str( 0 ) )
						node.set( 'totaldeltay', str( 0 ) )
						self.svgDataRead = True

	def UpdateSVGEggbotData( self, aNodeList ):
		if ( not self.svgDataRead ):
			for node in aNodeList:
				if node.tag == 'svg':
					self.UpdateSVGEggbotData( node )
				elif node.tag == inkex.addNS( 'eggbot', 'svg' ) or node.tag == 'eggbot':
					node.set( 'layer', str( self.svgLayer ) )
					node.set( 'node', str( self.svgNodeCount ) )
					node.set( 'lastpath', str( self.svgLastPath ) )
					node.set( 'lastpathnc', str( self.svgLastPathNC ) )
					node.set( 'totaldeltax', str( self.svgTotalDeltaX ) )
					node.set( 'totaldeltay', str( self.svgTotalDeltaY ) )
					self.svgDataRead = True

	def resumePlotSetup( self ):
		self.LayerFound = False
		if ( self.svgLayer < 101 ) and ( self.svgLayer >= 0 ):
			self.options.layernumber = self.svgLayer
			self.allLayers = False
			self.plotCurrentLayer = False
			self.LayerFound = True
		elif ( self.svgLayer == 12345 ):  # Plot all layers
			self.allLayers = True
			self.plotCurrentLayer = True
			self.LayerFound = True
		if ( self.LayerFound ):
			if ( self.svgNodeCount > 0 ):
				self.nodeTarget = self.svgNodeCount
				self.resumeMode = True
				if ( self.options.cancelOnly ):
					self.resumeMode = False
					self.penUp()   #Preemptively raise pen, before returning home.
					self.fPrevX = self.svgTotalDeltaX
					self.fPrevY = self.svgTotalDeltaY
					self.fX = 0
					self.fY = 0
					self.plotLineAndTime()
					self.svgLayer = 0
					self.svgNodeCount = 0
					self.svgLastPath = 0
					self.svgLastPathNC = 0
					self.svgTotalDeltaX = 0
					self.svgTotalDeltaY = 0

	def manualCommand( self ):
		"""Execute commands from the "manual" tab"""

		if self.options.manualType == "none":
			return
			
		if self.serialPort is None:
			return
			
		if self.options.manualType == "raise-pen":
			self.ServoSetupWrapper()
			self.penUp()

		elif self.options.manualType == "lower-pen":
			self.ServoSetupWrapper()
			self.penDown()

		elif self.options.manualType == "enable-motors":
			ebb_motion.sendEnableMotors(self.serialPort, 1) # 16X microstepping

		elif self.options.manualType == "disable-motors":
			self.sendDisableMotors()

		elif self.options.manualType == "version-check":
			strVersion = ebb_serial.query( self.serialPort, 'v\r' )
			inkex.errormsg( 'I asked the EBB for its version info, and it replied:\n ' + strVersion )

		elif self.options.manualType == "enable-engraver":
			if ( not self.options.engraving ):
				inkex.errormsg( gettext.gettext( "The engraver option is disabled. " + \
				" Please enable it first from the \"Options\" tab." ) )
			else:
				self.engraverOn()

		elif self.options.manualType == 'disable-engraver':
			self.engraverOffManual() #Force engraver off, even if it is not enabled.

		else:  # self.options.manualType is "walk-egg-motor" or "walk-pen-motor":
			if self.options.manualType == "walk-egg-motor":
				nDeltaX = self.options.WalkDistance
				nDeltaY = 0
			elif self.options.manualType == "walk-pen-motor":
				nDeltaY = self.options.WalkDistance
				nDeltaX = 0
			else:
				return
				
			ebb_motion.sendEnableMotors(self.serialPort, 1) # 16X microstepping
			
			#Query pen position: 1 up, 0 down (followed by OK)
			strVersion = ebb_serial.query( self.serialPort, 'QP\r' )
			
			if strVersion[0] == '0':
				self.fSpeed = self.options.penDownSpeed
			if strVersion[0] == '1':
				self.fSpeed = self.options.penUpSpeed

			if ( self.options.revPenMotor ):
				nDeltaY = -1 * nDeltaY
			if ( self.options.revEggMotor ):
				nDeltaX = -1 * nDeltaX
			
			nTime = 10000.00 / self.fSpeed * plot_utils.distance( nDeltaX, nDeltaY )
			nTime = int( math.ceil(nTime / 10.0))
			
			strOutput = ','.join( ['SM', str( nTime ), str( nDeltaY ), str( nDeltaX )] ) + '\r'
			#inkex.errormsg( 'strOutput:  ' + strOutput )

			ebb_serial.command( self.serialPort, strOutput )

	def setupCommand( self ):
		"""Execute commands from the "setup" tab"""

		if self.serialPort is None:
			return
		self.ServoSetupWrapper()
		if self.options.setupType == "align-mode":
			self.penUp()
			self.sendDisableMotors()
		else:
			ebb_motion.TogglePen(self.serialPort)
			
	def plotToEggBot( self ):
		'''Perform the actual plotting, if selected in the interface:'''
		#parse the svg data as a series of line segments and send each segment to be plotted

		if self.serialPort is None:
			return

		if self.options.startCentered and ( not self.getDocProps() ):
			# Cannot handle the document's dimensions!!!
			inkex.errormsg( gettext.gettext(
			'This document does not have valid dimensions.\r\r' +
			'Consider starting with the EggBot template, or ' +
			'setting the document size to 3200 px (wide) x 800 px (tall).\r\r' +
			'Document dimensions may be set in Inkscape with ' +
			'File > Document Properties.\r\rThe document dimensions must be unitless or have ' +
			'units of pixels (px) or percentages (%).   '	) )
			return

		# Viewbox handling
		# Also ignores the preserveAspectRatio attribute
		viewbox = self.svg.get( 'viewBox' )
		if viewbox:
			vinfo = viewbox.strip().replace( ',', ' ' ).split( ' ' )
			if ( float(vinfo[2]) != 0 ) and ( float(vinfo[3]) != 0 ):
				sx = self.svgWidth / float( vinfo[2] )
				sy = self.svgHeight / float( vinfo[3] )
				self.svgTransform = parseTransform( 'scale(%f,%f) translate(%f,%f)' % (sx, sy, -float( vinfo[0] ), -float( vinfo[1] ) ) )

		self.ServoSetup()
		ebb_motion.sendEnableMotors(self.serialPort, 1) # 16X microstepping
		
		# Ensure that the engraver is turned off for the time being
		# It will be turned back on when the first non-virtual pen-down occurs
		if self.options.engraving:
			self.engraverOff()


		try:
			# wrap everything in a try so we can for sure close the serial port
			#self.recursivelyTraverseSvg(self.document.getroot())
			self.penDownActivatesEngraver = True
			self.recursivelyTraverseSvg( self.svg, self.svgTransform )
			self.penUp()   #Always end with pen-up

			# Logically, we want to turn the engraver off here as well,
			# but we put that in our finally clause instead
			# self.engraverOff()

			# return to home, if returnToHome = True
			if ( ( not self.bStopped ) and self.options.returnToHome and ( self.ptFirst ) ):
				self.fX = self.ptFirst[0]
				self.fY = self.ptFirst[1]
				#self.penUp()
				self.nodeCount = self.nodeTarget    # enablesfpx return-to-home only option
				self.plotLineAndTime()
			#inkex.errormsg('Final node count: ' + str(self.svgNodeCount))  #Node Count - Debug option
			if ( not self.bStopped ):
				self.svgLayer = 0
				self.svgNodeCount = 0
				self.svgLastPath = 0
				self.svgLastPathNC = 0
				self.svgTotalDeltaX = 0
				self.svgTotalDeltaY = 0

		finally:
			# We may have had an exception and lost the serial port...
			self.penDownActivatesEngraver = False
			if ( not ( self.serialPort is None ) ) and ( self.options.engraving ):
				self.engraverOff()

	def recursivelyTraverseSvg( self, aNodeList,
			matCurrent=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
			parent_visibility='visible' ):
		"""
		Recursively traverse the svg file to plot out all of the
		paths.  The function keeps track of the composite transformation
		that should be applied to each path.

		This function handles path, group, line, rect, polyline, polygon,
		circle, ellipse and use (clone) elements.  Notable elements not
		handled include text.  Unhandled elements should be converted to
		paths in Inkscape.
		"""
		for node in aNodeList:
			if self.bStopped:
				return
			# Ignore invisible nodes
			v = node.get( 'visibility', parent_visibility )
			if v == 'inherit':
				v = parent_visibility
			if v == 'hidden' or v == 'collapse':
				continue

			# first apply the current matrix transform to this node's tranform
			matNew = composeTransform( matCurrent, parseTransform( node.get( "transform" ) ) )

			if node.tag == inkex.addNS( 'g', 'svg' ) or node.tag == 'g':

				self.penUp()
				if ( node.get( inkex.addNS( 'groupmode', 'inkscape' ) ) == 'layer' ):
					self.sCurrentLayerName = node.get( inkex.addNS( 'label', 'inkscape' ) )
					if not self.allLayers:
						self.DoWePlotLayer( self.sCurrentLayerName )
				self.recursivelyTraverseSvg( node, matNew, parent_visibility=v )

			elif node.tag == inkex.addNS( 'use', 'svg' ) or node.tag == 'use':

				# A <use> element refers to another SVG element via an xlink:href="#blah"
				# attribute.  We will handle the element by doing an XPath search through
				# the document, looking for the element with the matching id="blah"
				# attribute.  We then recursively process that element after applying
				# any necessary (x,y) translation.
				#
				# Notes:
				#  1. We ignore the height and width attributes as they do not apply to
				#     path-like elements, and
				#  2. Even if the use element has visibility="hidden", SVG still calls
				#     for processing the referenced element.  The referenced element is
				#     hidden only if its visibility is "inherit" or "hidden".

				refid = node.get( inkex.addNS( 'href', 'xlink' ) )
				if refid:
					# [1:] to ignore leading '#' in reference
					path = '//*[@id="%s"]' % refid[1:]
					refnode = node.xpath( path )
					if refnode:
						x = float( node.get( 'x', '0' ) )
						y = float( node.get( 'y', '0' ) )
						# Note: the transform has already been applied
						if ( x != 0 ) or (y != 0 ):
							matNew2 = composeTransform( matNew, parseTransform( 'translate(%f,%f)' % (x,y) ) )
						else:
							matNew2 = matNew
						v = node.get( 'visibility', v )
						self.recursivelyTraverseSvg( refnode, matNew2, parent_visibility=v )
					else:
						pass
				else:
					pass
			elif self.plotCurrentLayer:	#Skip subsequent tag checks unless we are plotting this layer.
				if node.tag == inkex.addNS( 'path', 'svg' ):
	
					# if we're in resume mode AND self.pathcount < self.svgLastPath,
					#    then skip over this path.
					# if we're in resume mode and self.pathcount = self.svgLastPath,
					#    then start here, and set
					# self.nodeCount equal to self.svgLastPathNC
	
					doWePlotThisPath = False 
					if (self.resumeMode):
						if ( self.pathcount < self.svgLastPath ):
							self.pathcount += 1
						elif ( self.pathcount == self.svgLastPath ):
							self.nodeCount = self.svgLastPathNC
							doWePlotThisPath = True 
					else:
						doWePlotThisPath = True
					if (doWePlotThisPath):
						self.pathcount += 1
						self.plotPath( node, matNew )
						if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
							self.svgLastPath += 1
							self.svgLastPathNC = self.nodeCount
	
				elif node.tag == inkex.addNS( 'rect', 'svg' ) or node.tag == 'rect':
	
					# Manually transform
					#
					#    <rect x="X" y="Y" width="W" height="H"/>
					#
					# into
					#
					#    <path d="MX,Y lW,0 l0,H l-W,0 z"/>
					#
					# I.e., explicitly draw three sides of the rectangle and the
					# fourth side implicitly
	
					doWePlotThisPath = False 
					if (self.resumeMode):
						if ( self.pathcount < self.svgLastPath ):
							self.pathcount += 1
						elif ( self.pathcount == self.svgLastPath ):
							self.nodeCount = self.svgLastPathNC
							doWePlotThisPath = True 
					else:
						doWePlotThisPath = True
					if (doWePlotThisPath):
						self.pathcount += 1
						newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
						x = float( node.get( 'x' ) )
						y = float( node.get( 'y' ) )
						w = float( node.get( 'width' ) )
						h = float( node.get( 'height' ) )
						s = node.get( 'style' )
						if s:
							newpath.set( 'style', s )
						t = node.get( 'transform' )
						if t:
							newpath.set( 'transform', t )
						a = []
						a.append( ['M ', [x, y]] )
						a.append( [' l ', [w, 0]] )
						a.append( [' l ', [0, h]] )
						a.append( [' l ', [-w, 0]] )
						a.append( [' Z', []] )
						newpath.set( 'd', simplepath.formatPath( a ) )
						self.plotPath( newpath, matNew )
						if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
							self.svgLastPath += 1
							self.svgLastPathNC = self.nodeCount
	
				elif node.tag == inkex.addNS( 'line', 'svg' ) or node.tag == 'line':
	
					# Convert
					#
					#   <line x1="X1" y1="Y1" x2="X2" y2="Y2/>
					#
					# to
					#
					#   <path d="MX1,Y1 LX2,Y2"/>
	
					doWePlotThisPath = False 
					if (self.resumeMode):
						if ( self.pathcount < self.svgLastPath ):
							self.pathcount += 1
						elif ( self.pathcount == self.svgLastPath ):
							self.nodeCount = self.svgLastPathNC
							doWePlotThisPath = True 
					else:
						doWePlotThisPath = True
					if (doWePlotThisPath):
						self.pathcount += 1
						newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
						x1 = float( node.get( 'x1' ) )
						y1 = float( node.get( 'y1' ) )
						x2 = float( node.get( 'x2' ) )
						y2 = float( node.get( 'y2' ) )
						s = node.get( 'style' )
						if s:
							newpath.set( 'style', s )
						t = node.get( 'transform' )
						if t:
							newpath.set( 'transform', t )
						a = []
						a.append( ['M ', [x1, y1]] )
						a.append( [' L ', [x2, y2]] )
						newpath.set( 'd', simplepath.formatPath( a ) )
						self.plotPath( newpath, matNew )
						if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
							self.svgLastPath += 1
							self.svgLastPathNC = self.nodeCount
	
				elif node.tag == inkex.addNS( 'polyline', 'svg' ) or node.tag == 'polyline':
	
					# Convert
					#
					#  <polyline points="x1,y1 x2,y2 x3,y3 [...]"/>
					#
					# to
					#
					#   <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...]"/>
					#
					# Note: we ignore polylines with no points
	
					pl = node.get( 'points', '' ).strip()
					if pl == '':
						pass
	
					doWePlotThisPath = False 
					if (self.resumeMode):
						if ( self.pathcount < self.svgLastPath ):
							self.pathcount += 1
						elif ( self.pathcount == self.svgLastPath ):
							self.nodeCount = self.svgLastPathNC
							doWePlotThisPath = True 
					else:
						doWePlotThisPath = True
					if (doWePlotThisPath):
						self.pathcount += 1
						pa = pl.split()
						if not len( pa ):
							continue
						# Issue 29: pre 2.5.? versions of Python do not have
						#    "statement-1 if expression-1 else statement-2"
						# which came out of PEP 308, Conditional Expressions
						#d = "".join( ["M " + pa[i] if i == 0 else " L " + pa[i] for i in range( 0, len( pa ) )] )
						d = "M " + pa[0]
						for i in range( 1, len( pa ) ):
							d += " L " + pa[i]
						newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
						newpath.set( 'd', d );
						s = node.get( 'style' )
						if s:
							newpath.set( 'style', s )
						t = node.get( 'transform' )
						if t:
							newpath.set( 'transform', t )
						self.plotPath( newpath, matNew )
						if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
							self.svgLastPath += 1
							self.svgLastPathNC = self.nodeCount
	
				elif node.tag == inkex.addNS( 'polygon', 'svg' ) or node.tag == 'polygon':
	
					# Convert
					#    <polygon points="x1,y1 x2,y2 x3,y3 [...]"/>
					# to
					#    <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...] Z"/>
					# Note: we ignore polygons with no points
	
					pl = node.get( 'points', '' ).strip()
					if pl == '':
						continue
	
					doWePlotThisPath = False 
					if (self.resumeMode):
						if ( self.pathcount < self.svgLastPath ):
							self.pathcount += 1
						elif ( self.pathcount == self.svgLastPath ):
							self.nodeCount = self.svgLastPathNC
							doWePlotThisPath = True 
					else:
						doWePlotThisPath = True
					if (doWePlotThisPath):
						self.pathcount += 1
						pa = pl.split()
						if not len( pa ):
							continue
						# Issue 29: pre 2.5.? versions of Python do not have
						#    "statement-1 if expression-1 else statement-2"
						# which came out of PEP 308, Conditional Expressions
						#d = "".join( ["M " + pa[i] if i == 0 else " L " + pa[i] for i in range( 0, len( pa ) )] )
						d = "M " + pa[0]
						for i in range( 1, len( pa ) ):
							d += " L " + pa[i]
						d += " Z"
						newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
						newpath.set( 'd', d );
						s = node.get( 'style' )
						if s:
							newpath.set( 'style', s )
						t = node.get( 'transform' )
						if t:
							newpath.set( 'transform', t )
						self.plotPath( newpath, matNew )
						if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
							self.svgLastPath += 1
							self.svgLastPathNC = self.nodeCount
	
	
				elif node.tag == inkex.addNS( 'ellipse', 'svg' ) or \
					node.tag == 'ellipse' or \
					node.tag == inkex.addNS( 'circle', 'svg' ) or \
					node.tag == 'circle':
	
						# Convert circles and ellipses to a path with two 180 degree arcs.
						# In general (an ellipse), we convert
						#
						#   <ellipse rx="RX" ry="RY" cx="X" cy="Y"/>
						#
						# to
						#
						#   <path d="MX1,CY A RX,RY 0 1 0 X2,CY A RX,RY 0 1 0 X1,CY"/>
						#
						# where
						#
						#   X1 = CX - RX
						#   X2 = CX + RX
						#
						# Note: ellipses or circles with a radius attribute of value 0 are ignored
						
						if node.tag == inkex.addNS( 'ellipse', 'svg' ) or node.tag == 'ellipse':
							rx = float( node.get( 'rx', '0' ) )
							ry = float( node.get( 'ry', '0' ) )
						else:
							rx = float( node.get( 'r', '0' ) )
							ry = rx
						if rx == 0 or ry == 0:
							pass
		
						doWePlotThisPath = False 
						if (self.resumeMode):
							if ( self.pathcount < self.svgLastPath ):
								self.pathcount += 1
							elif ( self.pathcount == self.svgLastPath ):
								self.nodeCount = self.svgLastPathNC
								doWePlotThisPath = True 
						else:
							doWePlotThisPath = True
						if (doWePlotThisPath):
							self.pathcount += 1
							cx = float( node.get( 'cx', '0' ) )
							cy = float( node.get( 'cy', '0' ) )
							x1 = cx - rx
							x2 = cx + rx
							d = 'M %f,%f ' % ( x1, cy ) + \
								'A %f,%f ' % ( rx, ry ) + \
								'0 1 0 %f,%f ' % ( x2, cy ) + \
								'A %f,%f ' % ( rx, ry ) + \
								'0 1 0 %f,%f' % ( x1, cy )
							newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
							newpath.set( 'd', d );
							s = node.get( 'style' )
							if s:
								newpath.set( 'style', s )
							t = node.get( 'transform' )
							if t:
								newpath.set( 'transform', t )
							self.plotPath( newpath, matNew )
							if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
								self.svgLastPath += 1
								self.svgLastPathNC = self.nodeCount
								
				elif node.tag == inkex.addNS( 'metadata', 'svg' ) or node.tag == 'metadata':
					pass
				elif node.tag == inkex.addNS( 'defs', 'svg' ) or node.tag == 'defs':
					pass
				elif node.tag == inkex.addNS( 'namedview', 'sodipodi' ) or node.tag == 'namedview':
					pass
				elif node.tag == inkex.addNS( 'eggbot', 'svg' ) or node.tag == 'eggbot':
					pass
				elif node.tag == inkex.addNS( 'WCB', 'svg' ) or node.tag == 'WCB':
					pass
				elif node.tag == inkex.addNS( 'title', 'svg' ) or node.tag == 'title':
					pass
				elif node.tag == inkex.addNS( 'desc', 'svg' ) or node.tag == 'desc':
					pass
				elif (node.tag == inkex.addNS( 'text', 'svg' ) or node.tag == 'text' or
					node.tag == inkex.addNS( 'flowRoot', 'svg' ) or node.tag == 'flowRoot'):
					if not self.warnings.has_key( 'text' ):
						inkex.errormsg( gettext.gettext( 'Warning: in layer "' + 
							self.sCurrentLayerName + '" unable to draw text; ' +
							'please convert it to a path first.  Consider using the ' +
							'Hershey Text extension which is located under the '+
							'"Render" category of extensions.' ) )
						self.warnings['text'] = 1
					pass
				elif node.tag == inkex.addNS( 'image', 'svg' ) or node.tag == 'image':
					if not self.warnings.has_key( 'image' ):
						inkex.errormsg( gettext.gettext( 'Warning: in layer "' + 
							self.sCurrentLayerName + '" unable to draw bitmap images; ' +
							'please convert them to line art first.  Consider using the "Trace bitmap..." ' +
							'tool of the "Path" menu.  Mac users please note that some X11 settings may ' +
							'cause cut-and-paste operations to paste in bitmap copies.' ) )
						self.warnings['image'] = 1
					pass
				elif node.tag == inkex.addNS( 'pattern', 'svg' ) or node.tag == 'pattern':
					pass
				elif node.tag == inkex.addNS( 'radialGradient', 'svg' ) or node.tag == 'radialGradient':
					# Similar to pattern
					pass
				elif node.tag == inkex.addNS( 'linearGradient', 'svg' ) or node.tag == 'linearGradient':
					# Similar in pattern
					pass
				elif node.tag == inkex.addNS( 'style', 'svg' ) or node.tag == 'style':
					# This is a reference to an external style sheet and not the value
					# of a style attribute to be inherited by child elements
					pass
				elif node.tag == inkex.addNS( 'cursor', 'svg' ) or node.tag == 'cursor':
					pass
				elif node.tag == inkex.addNS( 'color-profile', 'svg' ) or node.tag == 'color-profile':
					# Gamma curves, color temp, etc. are not relevant to single color output
					pass
				elif not isinstance( node.tag, basestring ):
					# This is likely an XML processing instruction such as an XML
					# comment.  lxml uses a function reference for such node tags
					# and as such the node tag is likely not a printable string.
					# Further, converting it to a printable string likely won't
					# be very useful.
					pass
				else:
					if not self.warnings.has_key( str( node.tag ) ):
						t = str( node.tag ).split( '}' )
						inkex.errormsg( gettext.gettext( 'Warning: in layer "' + 
							self.sCurrentLayerName + '" unable to draw <' + str( t[-1] ) +
							'> object, please convert it to a path first.' ) )
						self.warnings[str( node.tag )] = 1
					pass

	def DoWePlotLayer( self, strLayerName ):
		"""
		We are only plotting *some* layers. Check to see
		whether or not we're going to plot this one.

		First: scan first 4 chars of node id for first non-numeric character,
		and scan the part before that (if any) into a number

		Then, see if the number matches the layer.
		"""

		TempNumString = 'x'
		stringPos = 1
		CurrentLayerName = string.lstrip( strLayerName.encode( 'ascii', 'ignore' ) ) #remove leading whitespace
		
		# Look at layer name.  Sample first character, then first two, and
		# so on, until the string ends or the string no longer consists of
		# digit characters only.

		MaxLength = len( CurrentLayerName )
		if MaxLength > 0:
			while stringPos <= MaxLength:
				if str.isdigit( CurrentLayerName[:stringPos] ):
					TempNumString = CurrentLayerName[:stringPos] # Store longest numeric string so far
					stringPos = stringPos + 1
				else:
					break

		self.plotCurrentLayer = False    #Temporarily assume that we aren't plotting the layer
		if ( str.isdigit( TempNumString ) ):
			if ( self.svgLayer == int( float( TempNumString ) ) ):
				self.plotCurrentLayer = True	#We get to plot the layer!
				self.LayersPlotted += 1
		#Note: this function is only called if we are NOT plotting all layers.

	def getDocProps( self ):
		'''
		Get the document's height and width attributes from the <svg> tag.
		Use a default value in case the property is not present or is
		expressed in units of percentages.
		'''
		self.svgHeight = plot_utils.getLength( self, 'height', eggbot_conf.N_PAGE_HEIGHT )
		self.svgWidth = plot_utils.getLength( self, 'width', eggbot_conf.N_PAGE_WIDTH )
		if ( self.svgHeight == None ) or ( self.svgWidth == None ):
			return False
		else:
			return True

	def plotPath( self, path, matTransform ):
		'''
		Plot the path while applying the transformation defined
		by the matrix [matTransform].
		'''
		# turn this path into a cubicsuperpath (list of beziers)...

		d = path.get( 'd' )

		if len( simplepath.parsePath( d ) ) == 0:
			return

		p = cubicsuperpath.parsePath( d )

		# ...and apply the transformation to each point
		applyTransformToPath( matTransform, p )

		# p is now a list of lists of cubic beziers [control pt1, control pt2, endpoint]
		# where the start-point is the last point in the previous segment.
		for sp in p:

			plot_utils.subdivideCubicPath( sp, self.options.smoothness )
			nIndex = 0

			for csp in sp:

				if self.bStopped:
					return

				self.fX = 2 * float( csp[1][0] ) / self.step_scaling_factor
				self.fY = 2 * float( csp[1][1] ) / self.step_scaling_factor

				# store home
				if self.ptFirst is None:

					# if we should start at center, then the first line segment should draw from there
					if self.options.startCentered:
						self.fPrevX = self.svgWidth / ( self.step_scaling_factor )
						self.fPrevY = self.svgHeight / ( self.step_scaling_factor )

						self.ptFirst = ( self.fPrevX, self.fPrevY )
					else:
						self.ptFirst = ( self.fX, self.fY )
						
				if self.plotCurrentLayer:
					if nIndex == 0:
						if (plot_utils.distance(self.fX - self.fPrevX,self.fY - self.fPrevY) > eggbot_conf.MIN_GAP):
							# Only raise pen between two points if there is at least a 1 step gap between them.
							self.penUp()
							self.virtualPenIsUp = True
					elif nIndex == 1:
						self.penDown()
						self.virtualPenIsUp = False

				nIndex += 1

				if self.plotCurrentLayer:
					self.plotLineAndTime()
					self.fPrevX = self.fX
					self.fPrevY = self.fY

	def sendDisableMotors( self ):
		# Insist on turning the engraver off.  Otherwise, if it is on
		# and the pen is down, then the engraver's vibration may cause
		# the loose pen arm to start moving or the egg to start turning.
		self.engraverOffManual()
		ebb_motion.sendDisableMotors(self.serialPort)	

	def penUp( self ):
		self.virtualPenIsUp = True  # Virtual pen keeps track of state for resuming plotting.
		if (self.bPenIsUp != True): # Continue only if pen state is down (or unknown)
			if ( not self.resumeMode): # or if we're resuming.
				ebb_motion.sendPenUp(self.serialPort, self.options.penUpDelay )				
				if (self.options.penUpDelay  > 15):
					if self.options.tab != '"manual"':
						time.sleep(float(self.options.penUpDelay - 10)/1000.0)  #pause before issuing next command
				self.bPenIsUp = True

	def penDown( self ):
		self.virtualPenIsUp = False  # Virtual pen keeps track of state for resuming plotting.
		if (self.bPenIsUp != False):  # Continue only if pen state is up (or unknown)
			if ( not self.resumeMode ) and (not self.bStopped):  # skip if we're resuming or stopped
				self.bPenIsUp = False
				if self.penDownActivatesEngraver:
						self.engraverOn() # will check self.enableEngraver
				ebb_motion.sendPenDown(self.serialPort, self.options.penDownDelay )						
				if (self.options.penUpDelay  > 15):
					if self.options.tab != '"manual"':
						time.sleep(float(self.options.penDownDelay - 10)/1000.0)  #pause before issuing next command

	def engraverOff( self ):
		# Note: we don't bother checking self.engraverIsOn -- turn it off regardless
		# Reason being that we may not know the true hardware state
		if self.options.engraving:
			ebb_serial.command( self.serialPort, 'PO,B,3,0\r')		
			self.engraverIsOn = False
			
	def engraverOffManual( self ):
		# Turn off engraver, whether or not the engraver is enabled. 
		# This is only called by manual commands like "engraver off" and "motors off."
		ebb_serial.command( self.serialPort, 'PO,B,3,0\r')		
		self.engraverIsOn = False			
			
	def engraverOn( self ):
		if self.options.engraving and ( not self.engraverIsOn ):
			self.engraverIsOn = True
			ebb_serial.command( self.serialPort,  'PD,B,3,0\r')		#Added 6/6/2011, necessary.
			ebb_serial.command( self.serialPort,  'PO,B,3,1\r' )		

	def ServoSetupWrapper( self ):
		self.ServoSetup()
		strVersion = ebb_serial.query( self.serialPort,  'QP\r' ) #Query pen position: 1 up, 0 down (followed by OK)
		if strVersion[0] == '0':
			ebb_motion.sendPenDown(self.serialPort, 0)				
		else:
			ebb_motion.sendPenUp(self.serialPort, 0)				


	def ServoSetup( self ):
		# Pen position units range from 0% to 100%, which correspond to
		# a timing range of 6000 - 30000 in units of 1/(12 MHz).
		# 1% corresponds to 20 us, or 240 units of 1/(12 MHz).

		intTemp = 240 * ( self.options.penUpPosition + 25 )
		ebb_serial.command( self.serialPort,  'SC,4,' + str( intTemp ) + '\r' )	
		
		intTemp = 240 * ( self.options.penDownPosition + 25 )
		ebb_serial.command( self.serialPort,  'SC,5,' + str( intTemp ) + '\r'  )	

		# Servo speed units are in units of %/second, referring to the
		# percentages above.  The EBB takes speeds in units of 1/(12 MHz) steps
		# per 21 ms.  Scaling as above, 1% in 1 second corresponds to
		# 240 steps/s, which corresponds to 0.240 steps/ms, which corresponds
		# to 5.04 steps/21 ms.  Rounding this to 5 steps/21 ms is correct
		# to within 1 %.

		intTemp = 5 * self.options.ServoUpSpeed
		ebb_serial.command( self.serialPort,  'SC,11,' + str( intTemp ) + '\r' )	
		intTemp = 5 * self.options.ServoDownSpeed
		ebb_serial.command( self.serialPort,  'SC,12,' + str( intTemp ) + '\r' )	

	def stop( self ):
		self.bStopped = True

	def plotLineAndTime( self ):
		'''
		Send commands out the com port as a line segment (dx, dy) and a time (ms) the segment
		should take to implement
		'''

		if self.bStopped:
			return
		if ( self.fPrevX is None ):
			return

		nDeltaX = int( self.fX ) - int( self.fPrevX )
		nDeltaY = int( self.fY ) - int( self.fPrevY )

		if self.bPenIsUp:
			self.fSpeed = self.options.penUpSpeed
			
			if ( self.options.wraparound ):
				if ( nDeltaX > self.halfWrapSteps ):
					while ( nDeltaX > self.halfWrapSteps ):
						nDeltaX -= self.wrapSteps
				elif ( nDeltaX < -1 * self.halfWrapSteps ):
					while ( nDeltaX < -1 * self.halfWrapSteps  ):
						nDeltaX += self.wrapSteps

		else:
			self.fSpeed = self.options.penDownSpeed

		if ( plot_utils.distance( nDeltaX, nDeltaY ) > 0 ):
			self.nodeCount += 1

			if self.resumeMode:
				if ( self.nodeCount > self.nodeTarget ):
					self.resumeMode = False
					if ( not self.virtualPenIsUp ):
						self.penDown()
						self.fSpeed = self.options.penDownSpeed

			nTime = int( math.ceil( 1000.0 / self.fSpeed * plot_utils.distance( nDeltaX, nDeltaY ) ) )

			while ( ( abs( nDeltaX ) > 0 ) or ( abs( nDeltaY ) > 0 ) ):
				xd = nDeltaX
				yd = nDeltaY
				td = nTime
				if ( td < 1 ):
					td = 1		# don't allow zero-time moves.
					
				if (abs((float(xd) / float(td))) < 0.002):	
					xd = 0	#don't allow too-slow movements of this axis
				if (abs((float(yd) / float(td))) < 0.002):	
					yd = 0	#don't allow too-slow movements of this axis
						
				if ( not self.resumeMode ):
					if ( self.options.revPenMotor ):
						yd2 = yd
					else:
						yd2 = -yd
					if ( self.options.revEggMotor ):
						xd2 = -xd
					else:
						xd2 = xd

					self.svgTotalDeltaX += xd
					self.svgTotalDeltaY += yd
					ebb_motion.doXYMove( self.serialPort, xd2, yd2, td )
					if (td > 50):
						time.sleep(float(td - 50)/1000.0)  #pause before issuing next command
					
				nDeltaX -= xd
				nDeltaY -= yd
				nTime -= td

			strButton = ebb_motion.QueryPRGButton(self.serialPort)	# Query if button pressed
			if strButton == "":
				# Can't get a response from EBB, so
				# attempt to shut down in a way which allows user to continue.
				bNoResponseFromEbb = True
				strButton = '1'			# simulate pushed button for pause
			else:
				bNoResponseFromEbb = False
				
			if strButton[0] == '1': #button pressed, or simulated pressed because of communication error to allow resume
				self.svgNodeCount = self.nodeCount;
				if bNoResponseFromEbb:
					inkex.errormsg( 'Plot halted by communication error after node number ' + str( self.nodeCount ) + '.' )
				else:
					inkex.errormsg( 'Plot paused by button press after node number ' + str( self.nodeCount ) + '.' )
				inkex.errormsg( 'Use the "resume" feature to continue.' )
				self.engraverOff()
				self.bStopped = True
				return
e = EggBot()
e.affect()