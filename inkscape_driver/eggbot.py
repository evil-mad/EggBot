'''
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
'''

#TODO: Restrict individual segments to 5 s maximum & reduce timeout value
#TODO: Clean up serial search code, esp. for Windows
#TODO: Store save/resume data in inkscape document itself (?)
#TODO: Check why manual raise/lower pen don't use correct values ### Fix added, needs testing.

import inkex, cubicsuperpath, simplepath, cspsubdiv
from simpletransform import *
#import pathmodifier
from bezmisc import *
import gettext
import sys
import serial
import time
from math import sqrt
import string




F_DEFAULT_SPEED = 1
N_PEN_DOWN_DELAY = 400    # delay (ms) for the pen to go down before the next move
N_PEN_UP_DELAY = 400      # delay (ms) for the pen to up down before the next move
N_PEN_AXIS_STEPS = 1000   # steps the pen motor can move
N_EGG_AXIS_STEPS = 1000   # steps for the egg motor move in one revolution

N_PEN_UP_POS = 50      # Default pen-up position
N_PEN_DOWN_POS = 40      # Default pen-down position
N_SERVOSPEED = 50			# Default pen-lift speed
N_WALK_DEFAULT = 10		# Default steps for walking stepper motors 
N_DEFAULT_LAYER = 1			# Default inkscape layer
'''
if bDebug = True, create an HPGL file to show what is being plotted.
Pen up moves are shown in a different color if bDrawPenUpLines = True.
Try viewing the .hpgl file in a shareware program or create a simple viewer.
'''
bDebug = False 
miscDebug = False
bDrawPenUpLines = False
bDryRun = False # write the commands to a text file instead of the serial port

platform = sys.platform.lower()
if platform == 'win32':
    DEBUG_OUTPUT_FILE = 'C:/test.hpgl'
    DRY_RUN_OUTPUT_FILE = 'C:/dry_run.txt'
    MISC_OUTPUT_FILE = 'C:/misc.txt'	
    #STR_DEFAULT_COM_PORT = 'COM6'
else:
    import os #, tty
    DEBUG_OUTPUT_FILE = os.getenv('HOME') + '/test.hpgl'
    MISC_OUTPUT_FILE = os.getenv('HOME') + '/misc.txt'	
    DRY_RUN_OUTPUT_FILE = os.getenv('HOME') + '/dry_run.txt'
##    if platform == 'darwin':
##        ''' There's no good value for OS X '''
##        STR_DEFAULT_COM_PORT = '/dev/cu.usbmodem1a21'
##    elif platform == 'sunos':
##        ''' Untested: YMMV '''
##        STR_DEFAULT_COM_PORT = '/dev/term/0'
##    else:
##        ''' Works fine on Ubuntu; YMMV '''
##        STR_DEFAULT_COM_PORT = '/dev/ttyACM0'

'''
Break up a bezier curve into smaller curves, each of which
is approximately a straight line within a given tolerance
(the "smoothness" defined by [flat]).

This is a modified version of cspsubdiv.cspsubdiv(). I rewrote the recursive
call because it caused recursion-depth errors on complicated line segments.
'''
def subdivideCubicPath(sp,flat,i=1):

	while True:

		#m = 0

		while True:

			if i >= len(sp):
				return

			p0 = sp[i-1][1]
			p1 = sp[i-1][2]
			p2 = sp[i][0]
			p3 = sp[i][1]

			b = (p0,p1,p2,p3)

			if cspsubdiv.maxdist(b) > flat:
				break

			i += 1

		one, two = beziersplitatt(b,0.5)
		sp[i-1][2] = one[1]
		sp[i][0] = two[2]
		p = [one[2],one[3],two[1]]
		sp[i:1] = [p]

#class EggBot(pathmodifier.PathModifier):

class EggBot(inkex.Effect):
	def __init__(self):
		inkex.Effect.__init__(self)

		self.OptionParser.add_option("--smoothness",
                        action="store", type="float",
                        dest="smoothness", default=.2,
                        help="Smoothness of curves")
##		self.OptionParser.add_option( "--comPort",
##                        action="store", type="string",
##                        dest="comport", default=STR_DEFAULT_COM_PORT,
##                        help="USB COM port to connect eggbot.")
		self.OptionParser.add_option("--startCentered",
                        action="store", type="inkbool",
                        dest="startCentered", default=True,
                        help="Start plot with pen centered in the y-axis.")
		self.OptionParser.add_option("--returnToHome",
                        action="store", type="inkbool",
                        dest="returnToHome", default=True,
                        help="Return to home at end of plot.")
		self.OptionParser.add_option("--wraparound",
                        action="store", type="inkbool",
                        dest="wraparound", default=True,
                        help="Egg (x) axis wraps around-- take shortcuts!")
		self.OptionParser.add_option("--penUpSpeed",
                        action="store", type="int",
                        dest="penUpSpeed", default=F_DEFAULT_SPEED,
                        help="Speed (step/sec) while pen is up.")
		self.OptionParser.add_option("--penDownSpeed",
                        action="store", type="int",
                        dest="penDownSpeed", default=F_DEFAULT_SPEED,
                        help="Speed (step/sec) while pen is down.")
		self.OptionParser.add_option("--penDownDelay",
                        action="store", type="int",
                        dest="penDownDelay", default=N_PEN_DOWN_DELAY,
                        help="Delay after pen down (msec).")
		self.OptionParser.add_option("--penUpDelay",
                        action="store", type="int",
                        dest="penUpDelay", default=N_PEN_UP_DELAY,
                        help="Delay after pen up (msec).")
		self.OptionParser.add_option("--togglePenNow",
                        action="store", type="inkbool",
                        dest="togglePenNow", default=False,
                        help="Toggle the pen up/down on Apply.")
		self.OptionParser.add_option("--tab",
		                action="store", type="string",
						dest="tab", default="controls",
						help="The active tab when Apply was pressed") 
		self.OptionParser.add_option("--penUpPosition",
		                action="store", type="int",
						dest="penUpPosition", default=N_PEN_UP_POS,
						help="Position of pen when lifted") 				
		self.OptionParser.add_option("--ServoSpeed",
		                action="store", type="int",
						dest="ServoSpeed", default=N_SERVOSPEED,
						help="Rate of lifting/lowering pen ") 	
		self.OptionParser.add_option("--penDownPosition",
		                action="store", type="int",
						dest="penDownPosition", default=N_PEN_DOWN_POS,
						help="Position of pen when lowered")
		self.OptionParser.add_option("--layernumber",
		                action="store", type="int",
						dest="layernumber", default=N_DEFAULT_LAYER,
						help="Selected layer for multilayer plotting")
		self.OptionParser.add_option("--manualType",
		                action="store", type="string",
						dest="manualType", default="controls",
						help="The active tab when Apply was pressed") 
		self.OptionParser.add_option("--WalkDistance",
		                action="store", type="int",
						dest="WalkDistance", default=N_WALK_DEFAULT,
						help="Selected layer for multilayer plotting")	
		self.OptionParser.add_option("--cancelOnly",
                        action="store", type="inkbool",
                        dest="cancelOnly", default=False,
                        help="Cancel plot and return home only.")

		self.bPenIsUp = True
		self.virtualPenIsUp = False  #Keeps track of pen postion when stepping through plot before resuming
		self.fX = None
		self.fY = None
		self.fPrevX = None
		self.fPrevY = None
		self.ptFirst = None
		self.bStopped = False
		self.fSpeed = 1	
		self.resumeMode = False
		self.nodeCount = int(0)		#NOTE: python uses 32-bit ints.
		self.nodeTarget = int(0)
		self.pathcount = int(0)
		self.LayersPlotted = 0
		self.svgSerialPort = ''
		self.svgLayer = int(0)
		self.svgNodeCount = int(0) 
		self.svgDataRead = False
		self.svgLastPath = int(0)
		self.svgLastPathNC = int(0)
		self.svgTotalDeltaX = int(0)
		self.svgTotalDeltaY = int(0)
		
		self.nDeltaX = 0
		self.nDeltaY = 0
		self.DoubleStepSize = True
		
		try:
			import motor1600
		except ImportError:
			self.DoubleStepSize = False

	'''Main entry point: check to see which tab is selected, and act accordingly.'''									
	def effect(self):		#Pick main course of action, depending on active GUI tab when "Apply" was pressed.
		self.svg = self.document.getroot()
		self.CheckSVGforEggbotData()

		if self.options.tab == '"splash"': 
			self.allLayers = True
			self.plotCurrentLayer = True
			self.EggbotOpenSerial()
			self.svgNodeCount = 0
			self.svgLastPath = 0
			strButton = self.doRequest('QB\r') #Query if button pressed
			self.svgLayer = 12345;  # indicate that we are plotting all layers.
			self.plotToEggBot()


		elif self.options.tab == '"resume"': 
			self.EggbotOpenSerial() 
			strButton = self.doRequest('QB\r') #Query if button pressed
			self.resumePlotSetup()
			if self.resumeMode:
				self.plotToEggBot()
			elif (self.options.cancelOnly == True):
				pass
			else:
				inkex.errormsg(gettext.gettext("Truly sorry, there does not seem to be any in-progress plot to resume."))

		elif self.options.tab == '"layers"':
			self.allLayers = False 
			self.plotCurrentLayer = False
			self.LayersPlotted = 0
			self.svgLastPath = 0
			self.EggbotOpenSerial()
			strButton = self.doRequest('QB\r') #Query if button pressed
			self.svgNodeCount = 0;
			self.svgLayer = self.options.layernumber
			self.plotToEggBot()
			if (self.LayersPlotted == 0):
				inkex.errormsg(gettext.gettext("Truly sorry, but I did not find any named layers to plot."))
				
		elif self.options.tab == '"manual"': 
			self.EggbotOpenSerial()
			self.manualCommand()

			
		elif self.options.tab == '"timing"':
			self.EggbotOpenSerial()
			if self.serialPort is not None:
				self.ServoSetupWrapper()
		
		elif self.options.tab == '"options"':  
			self.EggbotOpenSerial()
			if self.serialPort is not None:
				self.ServoSetupWrapper()
				if self.options.togglePenNow: 
					self.doCommand('TP\r')		#Toggle pen

		self.svgDataRead = False
		self.UpdateSVGEggbotData(self.svg)
		self.EggbotCloseSerial() 
		return


	def CheckSVGforEggbotData(self):
		self.svgDataRead = False
		self.recursiveEggbotDataScan(self.svg)
		if (self.svgDataRead == False):    #if there is no eggbot data, add some:
			eggbotlayer = inkex.etree.SubElement(self.svg, 'eggbot')
			eggbotlayer.set('serialport', '')
			eggbotlayer.set('layer', str(0))
			eggbotlayer.set('node', str(0))
			eggbotlayer.set('lastpath', str(0))
			eggbotlayer.set('lastpathnc', str(0))
			eggbotlayer.set('totaldeltax', str(0))
			eggbotlayer.set('totaldeltay', str(0))

	def recursiveEggbotDataScan(self, aNodeList):
		if (self.svgDataRead != True):
			for node in aNodeList:
				if node.tag == 'svg':
					self.recursiveEggbotDataScan(node)
				elif node.tag == inkex.addNS('eggbot','svg') or node.tag == 'eggbot':
					self.svgSerialPort = node.get('serialport')
					self.svgLayer = int(node.get('layer'))
					self.svgNodeCount = int(node.get('node'))
					
					try:
						self.svgLastPath = int(node.get('lastpath'))
						self.svgLastPathNC = int(node.get('lastpathnc'))
						self.svgTotalDeltaX = int(node.get('totaldeltax'))
						self.svgTotalDeltaY = int(node.get('totaldeltay'))
						self.svgDataRead = True
					except:
						node.set('lastpath', str(0))
						node.set('lastpathnc', str(0))
						node.set('totaldeltax', str(0))
						node.set('totaldeltay', str(0))
						self.svgDataRead = True

	def UpdateSVGEggbotData(self, aNodeList):
		if (self.svgDataRead != True):
			for node in aNodeList:
				if node.tag == 'svg':
					self.UpdateSVGEggbotData(node)
				elif node.tag == inkex.addNS('eggbot','svg') or node.tag == 'eggbot':
					node.set('serialport', self.svgSerialPort)
					node.set('layer', str(self.svgLayer))
					node.set('node', str(self.svgNodeCount))
					node.set('lastpath', str(self.svgLastPath))
					node.set('lastpathnc', str(self.svgLastPathNC))
					node.set('totaldeltax', str(self.svgTotalDeltaX))
					node.set('totaldeltay', str(self.svgTotalDeltaY))
					self.svgDataRead = True

	def resumePlotSetup(self):
		self.LayerFound = False
		if (self.svgLayer < 101) and (self.svgLayer >= 0):
			self.options.layernumber = self.svgLayer
			self.allLayers = False 
			self.plotCurrentLayer = False
			self.LayerFound = True
		elif (self.svgLayer == 12345):  # Plot all layers
			self.allLayers = True
			self.plotCurrentLayer = True
			self.LayerFound = True
		if (self.LayerFound == True):
			if (self.svgNodeCount > 0):
				self.nodeTarget = self.svgNodeCount
				self.resumeMode = True
				if (self.options.cancelOnly == True):
					self.resumeMode = False
					self.fPrevX = self.svgTotalDeltaX
					self.fPrevY = self.svgTotalDeltaY
					self.fX = 0
					self.fY = 0
					self.plotLineAndTime()
					self.penUp()   #Always end with pen-up
					self.svgLayer = 0
					self.svgNodeCount = 0
					self.svgLastPath = 0
					self.svgLastPathNC = 0
					self.svgTotalDeltaX = 0
					self.svgTotalDeltaY = 0




	def manualCommand(self):	#execute commands from the "manual" tab
		#self.options.manualType 		

		if self.options.manualType == "none":
			return
		
		if self.serialPort is None:
			return
		
		self.ServoSetup()
		#walks are done at pen-down speed.  
		#TODO: Add auto-detect if pen is up or down.
		
		if self.options.manualType == "raise-pen":
			self.ServoSetupWrapper()
			self.penUp()
			
		elif self.options.manualType == "lower-pen":
			self.ServoSetupWrapper()
			self.penDown()

		elif self.options.manualType == "enable-motors":
			self.sendEnableMotors()
			
		elif self.options.manualType == "disable-motors":
			self.sendDisableMotors()

		elif self.options.manualType == "version-check":
			strVersion = self.doRequest('v\r')
			inkex.errormsg('I asked the EBB for its version info, and it replied:\n ' + strVersion )
		
		else:  # self.options.manualType is "walk-egg-motor" or "walk-pen-motor":
			if self.options.manualType == "walk-egg-motor":
				self.nDeltaX = self.options.WalkDistance
				self.nDeltaY = 0
			elif self.options.manualType == "walk-pen-motor":
				self.nDeltaY = self.options.WalkDistance
				self.nDeltaX = 0	
			else:
				return
			
			strVersion = self.doRequest('QP\r') #Query pen position: 1 up, 0 down (followed by OK)
			if strVersion[0] == '0':
				#inkex.errormsg('Pen is down' )
				self.fSpeed = self.options.penDownSpeed
			if strVersion[0] == '1':
				#inkex.errormsg('Pen is up' )
				self.fSpeed = self.options.penUpSpeed

			self.nTime = int(round(1000.0/self.fSpeed * distance(self.nDeltaX, self.nDeltaY)))
			strOutput = ','.join(['SM', str(self.nTime), str(self.nDeltaY), str(self.nDeltaX)]) + '\r'
			self.doCommand(strOutput)


	'''Perform the actual plotting, if selected in the interface:'''							
	def plotToEggBot(self):		#parse the svg data as a series of line segments and send each segment to be plotted
		
		if self.serialPort is None:
			return
			
		self.ServoSetup()
		
		if bDebug:
			self.debugOut = open(DEBUG_OUTPUT_FILE, 'w')
			if bDrawPenUpLines:
				self.debugOut.write('IN;SP1;')
			else:
				self.debugOut.write('IN;PD;')	
		try:    # wrap everything in a try so we can for sure close the serial port
			#self.recursivelyTraverseSvg(self.document.getroot())
			self.recursivelyTraverseSvg(self.svg)
			self.penUp()   #Always end with pen-up
			
			''' return to home, if returnToHome = True '''
			if ((self.bStopped == False) and self.options.returnToHome and (self.ptFirst != None)):
				self.fX = self.ptFirst[0]
				self.fY = self.ptFirst[1]
				#self.penUp()
				self.nodeCount = self.nodeTarget    # enablesfpx return-to-home only option
				self.plotLineAndTime()
			#inkex.errormsg('Final node count: ' + str(self.svgNodeCount))  #Node Count - Debug option
			if (self.bStopped == False):
				self.svgLayer = 0
				self.svgNodeCount = 0
				self.svgLastPath = 0
				self.svgLastPathNC = 0
				self.svgTotalDeltaX = 0
				self.svgTotalDeltaY = 0

		finally:
			return


##	Recursively traverse the svg file to plot out all of the
##	paths.  The function keeps track of the composite transformation
##	that should be applied to each path.
##
##	This function handles path, group and rect elements.  Most of these elements are
##	not used by Inkscape and aren't handled: clone, circle, ellipse, line, polyline,
##	polygon, text.  Those should be converted to paths in Inkscape.


	def recursivelyTraverseSvg(self, aNodeList, matCurrent = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]):
		for node in aNodeList:

			''' first apply the current matrix transform to this node's tranform '''
			matNew = composeTransform(matCurrent, parseTransform(node.get("transform")))
			if node.tag == inkex.addNS('g','svg') or node.tag == 'g':
				self.penUp()
				if (node.get(inkex.addNS('groupmode', 'inkscape')) == 'layer'):
					if self.allLayers == False:
						#inkex.errormsg('Plotting layer named: ' + node.get(inkex.addNS('label', 'inkscape')))
						self.DoWePlotLayer(node.get(inkex.addNS('label', 'inkscape')))
				self.recursivelyTraverseSvg(node, matNew)
			elif node.tag == inkex.addNS('use','svg') or node.tag == 'use':
				# HANDLE CLONES!
				pass
			elif node.tag == inkex.addNS('path','svg'):

				self.pathcount += 1
				#if we're in resume mode AND self.pathcount < self.svgLastPath, then skip over this path.
				#if we're in resume mode and self.pathcount = self.svgLastPath, then start here, and set
				# self.nodeCount equal to self.svgLastPathNC
				


				if self.resumeMode and (self.pathcount == self.svgLastPath):
					self.nodeCount = self.svgLastPathNC

				if self.resumeMode and (self.pathcount < self.svgLastPath):
					pass
				else: 
					self.plotPath(node, matNew)
					if (self.bStopped == False):	#an "index" for resuming plots quickly-- record last complete path
						self.svgLastPath += 1
						self.svgLastPathNC = self.nodeCount
					
			elif node.tag == inkex.addNS('pattern','svg') or node.tag == 'pattern':
				pass            
			elif node.tag == inkex.addNS('metadata','svg') or node.tag == 'metadata':
				pass
			elif node.tag == inkex.addNS('defs','svg') or node.tag == 'defs':
				pass
			elif node.tag == inkex.addNS('namedview','sodipodi') or node.tag == 'namedview':
				pass
			elif node.tag == inkex.addNS('eggbot','svg') or node.tag == 'eggbot':
				pass
			elif node.tag == inkex.addNS('text','svg') or node.tag == 'text':
				inkex.errormsg('Warning: unable to draw text, please convert it to a path first.')
				pass
			else:
				inkex.errormsg('Warning: unable to draw object, please convert it to a path first.')
				pass

	#We are only plotting *some* layers. Check to see
	#   whether or not we're going to plot this one.
	#
	# First: scan first 4 chars of node id for first non-numeric character,
	#  and scan the part before that (if any) into a number
	#
	# Then, see if the number matches the layer.
	
	#	self.plotCurrentLayer =  False

	def DoWePlotLayer(self, strLayerName):
		TempNumString = 'x'
		stringPos = 1
		CurrentLayerName = string.lstrip(strLayerName) #remove leading whitespace
		
		#Look at layer name.  Sample first character, then first two, and
		# so on, until the string ends or the string no longer consists of
		# digit characters only.
		
		MaxLength = len(CurrentLayerName)
		if MaxLength > 0:
			while stringPos <= MaxLength:
				if str.isdigit( CurrentLayerName[:stringPos]):
					TempNumString = CurrentLayerName[:stringPos] # Store longest numeric string so far
					stringPos = stringPos + 1
				else:
					break
		
		self.plotCurrentLayer =  False    #Temporarily assume that we aren't plotting the layer
		if (str.isdigit(TempNumString)):
			if (self.svgLayer == int(float(TempNumString))):
				self.plotCurrentLayer =  True	#We get to plot the layer!
				self.LayersPlotted += 1
		#Note: this function is only called if we are NOT plotting all layers.

	'''
	Plot the path while applying the transformation defined
	by the matrix [matTransform].
	'''
	def plotPath(self, path, matTransform):
		''' turn this path into a cubicsuperpath (list of beziers)... '''
		d = path.get('d')

		if len(simplepath.parsePath(d)) == 0:
			return

		p = cubicsuperpath.parsePath(d)

		''' ...and apply the transformation to each point '''
		applyTransformToPath(matTransform, p)

		'''
		p is now a list of lists of cubic beziers [control pt1, control pt2, endpoint]
		where the start-point is the last point in the previous segment.
		'''
		for sp in p:

			subdivideCubicPath(sp, self.options.smoothness)
			nIndex = 0

			for csp in sp:

				if self.bStopped:
					return

				if self.plotCurrentLayer:
					if nIndex == 0:
						self.penUp()
						self.virtualPenIsUp = True
					elif nIndex == 1:
						self.penDown()
						self.virtualPenIsUp = False

				nIndex += 1

				if (self.DoubleStepSize == True):
					self.fX = float(csp[1][0]) / 2.0
					self.fY = float(csp[1][1]) / 2.0
				else:
					self.fX = float(csp[1][0])
					self.fY = float(csp[1][1])

				''' store home '''
				if self.ptFirst is None:

					''' if we should start at center, then the first line segment should draw from there '''
					if self.options.startCentered:
						self.fPrevX = self.fX
						#self.fPrevY = N_PEN_AXIS_STEPS / 2.0
						
						if (self.DoubleStepSize == True):
							self.fPrevY = N_PEN_AXIS_STEPS / 4.0
						else:
							self.fPrevY = N_PEN_AXIS_STEPS / 2.0

						self.ptFirst = (self.fPrevX, self.fPrevY)
					else:
						self.ptFirst = (self.fX, self.fY)
						
				if self.plotCurrentLayer:
					self.plotLineAndTime()
					self.fPrevX = self.fX
					self.fPrevY = self.fY

	def sendEnableMotors(self):
		self.doCommand('EM,1,1\r')

	def sendDisableMotors(self):
		self.doCommand('EM,0,0\r')

	def penUp(self):
		if ((self.resumeMode != True) or (self.virtualPenIsUp != True)):
			self.doCommand('SP,1\r')
			self.doCommand('SM,' + str(self.options.penUpDelay) + ',0,0\r') # pause for pen to go up
			self.bPenIsUp = True
		self.virtualPenIsUp = True

	def penDown(self):
		self.virtualPenIsUp = False  # Virtual pen keeps track of state for resuming plotting.
		if (self.resumeMode != True):
			self.doCommand('SP,0\r')
			self.doCommand('SM,' + str(self.options.penDownDelay) + ',0,0\r') # pause for pen to go down
			self.bPenIsUp = False

	def ServoSetupWrapper(self):
		self.ServoSetup()
		strVersion = self.doRequest('QP\r') #Query pen position: 1 up, 0 down (followed by OK)
		if strVersion[0] == '0':
			#inkex.errormsg('Pen is down' )
			self.doCommand('SP,0\r') #Lower Pen
		else:
			self.doCommand('SP,1\r') #Raise pen
		
	def ServoSetup(self):
		# Pen position units range from 0% to 100%, which correspond to
		# a timing range of 6000 - 30000 in units of 1/(12 MHz).
		# 1% corresponds to 20 us, or 240 units of 1/(12 MHz).
		
		intTemp = 240 * (self.options.penUpPosition + 25)
		self.doCommand('SC,4,' + str(intTemp) + '\r')
		intTemp = 240 * (self.options.penDownPosition + 25)
		self.doCommand('SC,5,' + str(intTemp) + '\r')
		
		# Servo speed units are in units of %/second, referring to the
		# percentages above.  The EBB takes speeds in units of 1/(12 MHz) steps
		# per 21 ms.  Scaling as above, 1% in 1 second corresponds to
		# 240 steps/s, which corresponds to 0.240 steps/ms, which corresponds
		# to 5.04 steps/21 ms.  Rounding this to 5 steps/21 ms is correct
		# to within 1 %.
		
		intTemp = 5 * self.options.ServoSpeed
		self.doCommand('SC,10,' + str(intTemp) + '\r')
		#inkex.errormsg('Setting up Servo Motors!')


	def stop(self):
		self.bStopped = True

	'''
	Send commands out the com port as a line segment (dx, dy) and a time (ms) the segment
	should take to implement
	'''
	def plotLineAndTime(self):
		if self.bStopped:
			return
		if (self.fPrevX is None):
			return

		self.nDeltaX = int(self.fX) - int(self.fPrevX)
		self.nDeltaY = int(self.fY) - int(self.fPrevY)

		if self.bPenIsUp:
			self.fSpeed = self.options.penUpSpeed
			
			if (self.options.wraparound == True):
				if (self.DoubleStepSize == True):
					if (self.nDeltaX > 800):
						self.nDeltaX -= 1600
					elif (self.nDeltaX < -800):
						self.nDeltaX += 1600
				else:
					if (self.nDeltaX > 1600):
						self.nDeltaX -= 3200
					elif (self.nDeltaX < -1600):
						self.nDeltaX += 3200

		else:
			self.fSpeed = self.options.penDownSpeed


		if (distance(self.nDeltaX, self.nDeltaY) > 0):
			self.nodeCount += 1
			
			if self.resumeMode:
				if (self.nodeCount > self.nodeTarget):
					self.resumeMode = False
					#inkex.errormsg('First node plotted will be number: ' + str(self.nodeCount))
					if (self.virtualPenIsUp != True):
						self.penDown()
						self.fSpeed = self.options.penDownSpeed

			nTime = int(math.ceil(1000/self.fSpeed * distance(self.nDeltaX, self.nDeltaY)))


			while ((abs(self.nDeltaX) > 0) or (abs(self.nDeltaY) > 0)):
				if (nTime > 750):
					xd = int(round((750.0 * self.nDeltaX)/nTime))
					yd = int(round((750.0 * self.nDeltaY)/nTime))
					td = int(750)
				else:
					xd = self.nDeltaX
					yd = self.nDeltaY
					td = nTime
					if (td < 1):
						td = 1		# don't allow zero-time moves.

				if (self.resumeMode != True):
					strOutput = ','.join(['SM', str(td), str(-yd), str(xd)]) + '\r'
					self.svgTotalDeltaX += xd
					self.svgTotalDeltaY += yd
					self.doCommand(strOutput)
				
				self.nDeltaX -= xd
				self.nDeltaY -= yd
				nTime -= td
				
			#self.doCommand('NI\r')  #Increment node counter on EBB
			strButton = self.doRequest('QB\r') #Query if button pressed
			if strButton[0] == '0':
				pass #button not pressed
			else:
				self.svgNodeCount = self.nodeCount;
				inkex.errormsg('Plot paused by button press after segment number ' + str(self.nodeCount) + '.')
				inkex.errormsg('Use the "resume" feature to continue.' )
				#self.penUp()  # Should be redundant...
				self.bStopped = True
				return
				
		''' note: the pen-motor is first, and it corresponds to the y-axis on-screen '''


	def EggbotOpenSerial(self):
		if not bDryRun:
			self.serialPort = self.getSerialPort()
		else:
			self.serialPort = open(DRY_RUN_OUTPUT_FILE, 'w')	
			
		if self.serialPort is None:
			inkex.errormsg(gettext.gettext("Unable to find an Eggbot on any serial port. :("))

	def EggbotCloseSerial(self):
		try:
			if self.serialPort != None:
				self.serialPort.flush()
				self.serialPort.close()
			if bDebug:
				self.debugOut.close()
		finally:
			self.serialPort = None
			return



	'''
    look at COM1 to COM20 and return a SerialPort object
    for the first port with an EBB (eggbot board).

    YOU are responsible for closing this serial port!
    '''
	def testSerialPort(self, strComPort):
		try:
			serialPort = serial.Serial(strComPort, timeout=1) # 1 second timeout!

			serialPort.setRTS()  # ??? remove
			serialPort.setDTR()  # ??? remove
			serialPort.flushInput()
			serialPort.flushOutput()

			time.sleep(0.1)

			serialPort.write('v\r')
			strVersion = serialPort.readline()

			if strVersion != None and strVersion.startswith('EBB'):
				# do version control here to check the firmware...
				return serialPort
			serialPort.close()
		except serial.SerialException:
			pass
		return None
	
	def getSerialPort(self):
		if platform == 'win32':
			for i in range(1, 100):
				strComPort = 'COM' + str(i)
				serialPort = self.testSerialPort(strComPort)		
				if serialPort != None:
					return serialPort
		else:
			if platform == 'darwin':
				strDir = '/dev'
				strPrefix = 'cu.usbmodem'
			elif platform == 'sunos5':
				strDir = '/dev/term'
				strPrefix = None
			else:
				strDir = '/dev'
				strPrefix = 'ttyACM'

			device_list = os.listdir(strDir)
			# Before searching, first check to see if the
			# last known serial port is still good.
			if self.svgSerialPort in device_list:
				serialPort = self.testSerialPort(self.svgSerialPort)
				if serialPort != None:
					return serialPort

			for device in device_list:
				if strPrefix != None:
					if not device.startswith(strPrefix):
						continue
				strComPort = strDir + '/' + device
				serialPort = self.testSerialPort(strComPort)
				if serialPort != None:
					self.svgSerialPort = strComPort
					return serialPort
		return None

	def doCommand(self, cmd):
		try:
			self.serialPort.write(cmd)
			response = self.serialPort.readline()
			if (response != 'OK\r\n'):
				if (response != ''):
					inkex.errormsg('After command ' + cmd + ',')
					inkex.errormsg('Received bad response from EBB: ' + str(response) + '.')
					#inkex.errormsg('BTW:: Node number is ' + str(self.nodeCount) + '.')

				else:
					inkex.errormsg('EBB Serial Timeout.')

		except:
			pass

	def doRequest(self, cmd):
		response = ''
		try:
			self.serialPort.write(cmd)
			response = self.serialPort.readline()
			responseDummy = self.serialPort.readline() #read in extra blank/OK line
		except:
			inkex.errormsg(gettext.gettext("Error reading serial data."))

		return response	
'''
Pythagorean theorem!
'''
def distance(x, y):
   return sqrt(x*x + y*y)

e = EggBot()
#e.affect(output=False)
e.affect()