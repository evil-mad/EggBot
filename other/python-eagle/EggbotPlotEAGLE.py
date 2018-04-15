# coding=utf-8
import time
from math import sqrt

import thread

N_PEN_AXIS_STEPS = 800  # steps the pen motor can move
N_EGG_AXIS_STEPS = 3200  # steps for the egg motor move in one revolution


class EggbotPlotEAGLE:

    def __init__(self, parent):
        self.parent = parent
        self.serialPort = parent.serialPort

    def reset(self):
        self.fX = None
        self.fY = None
        self.fPrevX = None
        self.fPrevY = None
        self.bPenIsUp = True
        self.ptFirst = None
        self.bStartingFirstLayer = True

    def plot(self):

        if self.serialPort is None or not self.serialPort.isConnected():
            return

        self.reset()
        self.initializeServo()

        """ Start the plot function on a new thread so the gui still runs """
        args = ('filename',)
        thread.start_new_thread(self.start, args)

    """
    Send the EBB the up, down, and speed settings for the servo.
    """

    def initializeServo(self):
        self.serialPort.sendServoUpSetting(self.parent.nPenUpPos)
        self.serialPort.sendServoDownSetting(self.parent.nPenDownPos)
        self.serialPort.sendServoSpeedSetting(self.parent.nPenRaiseSpeed)

    """ start the recursive calls to plot the svg file, and update the gui when plot is finished """

    def start(self, filen):
        try:
            self.penUp(True)  # force a pen up at the start
            """ store home """
            self.fPrevX = N_EGG_AXIS_STEPS
            self.fX = 0
            self.fPrevY = N_PEN_AXIS_STEPS / 2
            self.ptFirst = (self.fPrevX, self.fPrevY)

            self.plotfile(filen)
            self.serialPort.sendPenUp()
            self.penUp(True)  # force a pen up at the end

            if self.parent.bReturnToHome and self.ptFirst is not None:
                (self.fX, self.fY) = self.ptFirst
                self.plotLineAndTime()  # return to home line

            self.parent.plotHasFinished()

        except Exception as ex:
            self.parent.reportError('Exception during plot: ' + str(ex))

    """
    Plot a simple pen plotter style file containing commands of the form
    C<pennumber>        ;; select pen
    M<x>,<y>            ;; Move to x,y with pen up.
    D<x>,<y>            ;; Move to x,y with pen down.
    x and y are in steps.  EAGLE (and probably other programs) can be
      coerced into generating this format of file.
    The main purpose of this program is to convert absolute coordinates
    to the relative step counts and timing needed by eggbot.
    """

    def plotfile(self, filen):
        self.infile = open('/tmp/plot.out', 'r')
        for line in self.infile:
            """ check if the user has hit the "stop" button """
            if self.parent.isStopped():
                return
            if self.parent.isPaused():
                while self.parent.isPaused():
                    time.sleep(.5)  # wait half a second before checking again

                """ make sure the pen is still in the right start before we start """
                if self.bPenIsUp:
                    self.penUp(True)
                else:
                    self.penDown(True)

            # print line[:-1]
            if line[0] == 'C':
                """ pause between layers, but don't pause before the first layer """
                if self.bStartingFirstLayer:
                    self.bStartingFirstLayer = False
                else:
                    self.parent.pauseForPenChange(line)

            elif line[0] == 'M':
                self.fX = N_EGG_AXIS_STEPS - int(line.split(',')[0][1:])
                self.fY = int(line.split(',')[1])
                # print " Move   ", self.fX, self.fY
                self.penUp()
                self.plotLineAndTime()
                self.fPrevX = self.fX
                self.fPrevY = self.fY
            elif line[0] == 'D':
                self.fX = N_EGG_AXIS_STEPS - int(line.split(',')[0][1:])
                self.fY = int(line.split(',')[1])
                # print " Draw   ", self.fX, self.fY
                self.penDown()
                self.plotLineAndTime()
                self.fPrevX = self.fX
                self.fPrevY = self.fY
            else:
                # Nothing yet
                pass

    def penUp(self, bForcePenUp=False):
        if (not self.bPenIsUp) or bForcePenUp:
            self.serialPort.sendPenUp()
            self.bPenIsUp = True
            self.serialPort.sendPause(self.parent.nPenUpDelay)

    def penDown(self, bForcePenDown=False):
        if self.bPenIsUp or bForcePenDown:
            self.serialPort.sendPenDown()
            self.bPenIsUp = False
            self.serialPort.sendPause(self.parent.nPenDownDelay)

    """
    Send commands out the com port as a line segment (dx, dy) and a time (ms) the segment
    should take to implement
    """

    def plotLineAndTime(self):

        if self.fPrevX is None:
            return

        if self.bPenIsUp:
            nSpeed = self.parent.nPenUpSpeed
        else:
            nSpeed = self.parent.nPenDownSpeed

        nDeltaX = int(self.fX) - int(self.fPrevX)
        nDeltaY = int(self.fY) - int(self.fPrevY)

        nTime = int(round(1000.0 / nSpeed * distance(nDeltaX, nDeltaY)))

        self.serialPort.sendMove(nDeltaX, nDeltaY, nTime)

    """ overload this function since we don't need to access the svg document """

    def uniqueId(self, prefix):
        return "1"


"""
Pythagorean theorem
"""


def distance(x, y):
    return sqrt(x * x + y * y)
