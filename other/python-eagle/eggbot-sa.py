# coding=utf-8
import os

import EggbotGuiConfiguration
import EggbotGuiManualControl
import EggbotGuiMenu
import EggbotPlotEAGLE
import EggbotSerial
import wx
from configobj import ConfigObj

ID_STARTPAUSE_BUTTON = 1001
ID_STOP_BUTTON = 1002

CONFIG_FILE_NAME = 'eggbot.cfg'

""" an "enum" to represent the current state """
RUNNING, PAUSED, STOPPED = range(3)


class EggbotGui(wx.Frame):
    ID_TIMER = 30  # ms

    def __init__(self, docRoot):

        wx.Frame.__init__(self, None, wx.ID_ANY, "Eggbot Plotter")

        self.initializeVariables()
        self.CreateStatusBar()
        self.menu = EggbotGuiMenu.EggbotGuiMenu(self)
        self.serialPort = EggbotSerial.EggbotSerial()
        self.EAGLEPlotter = EggbotPlotEAGLE.EggbotPlotEAGLE(self)
        self.docRoot = docRoot
        self.createButtons()

        guiRect = self.GetScreenRect()

        self.config = EggbotGuiConfiguration.EggbotGuiConfiguration(self)
        self.config.Move(guiRect.GetTopRight())
        self.config.Show(True)

        configRect = self.config.GetScreenRect()

        self.manualControl = EggbotGuiManualControl.EggbotGuiManualControl(self)
        self.manualControl.Show(True)
        self.manualControl.Move(configRect.GetBottomLeft())

    def createButtons(self):

        self.SetAutoLayout(True)

        panel = wx.Panel(self)
        vbox = wx.BoxSizer(wx.VERTICAL)

        if self.serialPort.getCurrentPort() is None:
            strStatus = "\nNo eggbot connected\n\n"
        else:
            strStatus = "\nConnected to " + self.serialPort.getCurrentPort() + "\n"

            strEbbVersion = self.serialPort.getEggbotVersion()

            if strEbbVersion is None:
                strEbbVersion = "Eggbot firmware not found"

            strStatus += strEbbVersion + "\n"

        self.messageText = wx.StaticText(panel, -1, strStatus, style=wx.ALIGN_CENTER)
        vbox.Add(self.messageText, flag=wx.ALIGN_CENTER)

        hbox = wx.BoxSizer(wx.HORIZONTAL)

        self.startPauseButton = wx.Button(panel, ID_STARTPAUSE_BUTTON, 'Start')
        self.Bind(wx.EVT_BUTTON, self.onStartOrPause, self.startPauseButton)
        hbox.Add(self.startPauseButton)

        self.stopButton = wx.Button(panel, ID_STOP_BUTTON, 'Stop')
        self.Bind(wx.EVT_BUTTON, self.onStop, self.stopButton)
        hbox.Add(self.stopButton)

        vbox.Add(hbox, flag=wx.ALIGN_CENTER)

        panel.SetSizerAndFit(vbox)
        self.SetClientSize(panel.GetSize())
        self.Show(True)

    def __del__(self):
        if self.serialPort is not None:
            self.serialPort.__del__()

    def onStartOrPause(self, event):
        if self.eCurrentState == STOPPED:
            """ start plot """
            self.eCurrentState = RUNNING
            self.startPauseButton.SetLabel('Pause')
            self.EAGLEPlotter.plot()

        elif self.eCurrentState == PAUSED:
            """ unpause plot """
            self.eCurrentState = RUNNING
            self.startPauseButton.SetLabel('Pause')

        else:
            """ pause plot """
            self.eCurrentState = PAUSED
            self.startPauseButton.SetLabel('Resume')

    def onStop(self, event):
        if self.eCurrentState == STOPPED:
            pass
        else:
            self.eCurrentState = STOPPED
            self.startPauseButton.SetLabel('Start')

            """ reset variables? """

    def isStopped(self):
        return self.eCurrentState == STOPPED

    def isPaused(self):
        return self.eCurrentState == PAUSED

    def onTimer(self, event):
        if event.GetId() == EggbotGui.ID_TIMER:
            pass
        else:
            event.Skip()

    def sendServoInitialization(self):
        if self.eCurrentState != RUNNING and self.serialPort.isConnected():
            self.EAGLEPlotter.initializeServo()
            pass

    def sendManualPenUp(self):
        if self.eCurrentState != RUNNING and self.serialPort.isConnected():
            self.serialPort.sendPenUp()

    def sendManualPenDown(self):
        if self.eCurrentState != RUNNING and self.serialPort.isConnected():
            self.serialPort.sendPenDown()

    def sendManualMove(self, nXSteps, nYSteps):
        if self.eCurrentState != RUNNING and self.serialPort.isConnected():
            self.serialPort.sendMove(nXSteps, nYSteps, 5 * (abs(nXSteps) + abs(nYSteps)))

    def reconnectToEggbot(self):
        if self.serialPort is not None:
            self.serialPort.reconnectToEggbot()
        else:
            self.serialPort = EggbotSerial.EggbotSerial()

    def initializeVariables(self):
        self.eCurrentState = STOPPED
        self.bMotorsEnabled = True

        """ try to load parameters from the file """
        if os.path.exists('eggbot.cfg'):
            try:
                config = ConfigObj(CONFIG_FILE_NAME)
                self.nPenDownSpeed = int(config['penDownSpeed'])
                self.nPenUpSpeed = int(config['penUpSpeed'])
                self.nPenDownDelay = int(config['penDownDelay'])
                self.nPenUpDelay = int(config['penUpDelay'])
                self.nSmoothness = int(config['smoothness'])
                self.bPauseBetweenLayers = (config['pauseBetweenLayers'] == 'True')  # simple boolean cast
                self.bReturnToHome = (config['returnToHome'] == 'True')
                self.bCenterPlot = (config['centerPlot'] == 'True')
                self.nPenUpPos = int(config['penUpPos'])
                self.nPenDownPos = int(config['penDownPos'])
                self.nPenRaiseSpeed = int(config['penRaiseSpeed'])
                return

            except:
                pass

        """ if no file exists or it has an error, just use the defaults """
        self.nPenDownSpeed = 200
        self.nPenUpSpeed = 400
        self.nPenDownDelay = 4
        self.nPenUpDelay = 4
        self.nSmoothness = 100  # %
        self.bReturnToHome = True
        self.bPauseBetweenLayers = True
        self.bCenterPlot = False
        self.nPenUpPos = 13500
        self.nPenDownPos = 11000
        self.nPenRaiseSpeed = 150

    """ store the current settings """

    def storeConfiguration(self):
        config = ConfigObj(CONFIG_FILE_NAME)
        config['penDownSpeed'] = self.nPenDownSpeed
        config['penUpSpeed'] = self.nPenUpSpeed
        config['penDownDelay'] = self.nPenDownDelay
        config['penUpDelay'] = self.nPenUpDelay
        config['smoothness'] = self.nSmoothness
        config['pauseBetweenLayers'] = self.bPauseBetweenLayers
        config['returnToHome'] = self.bReturnToHome
        config['centerPlot'] = self.bCenterPlot
        config['penUpPos'] = self.nPenUpPos
        config['penDownPos'] = self.nPenDownPos
        config['penRaiseSpeed'] = self.nPenRaiseSpeed
        config.write()

    def pauseForPenChange(self, strLayerName):
        if strLayerName is None:
            strLayerName = ''

        okDialog = wx.MessageDialog(self, 'About to plot new pen "' + strLayerName + '", would you like ' +
                                    'to change pens?  Press okay to continue.', '', wx.OK)
        okDialog.ShowModal()
        okDialog.Destroy()

    """ end of plot, same as user hitting stop """

    def plotHasFinished(self):
        self.onStop(None)

    def toggleDisableMotors(self):
        if self.eCurrentState != RUNNING:
            if self.bMotorsEnabled:
                self.bMotorsEnabled = False
                self.serialPort.sendDisableMotors()
            else:
                self.bMotorsEnabled = True
                self.serialPort.sendEnableMotors()

    def reportError(self, strError):
        errDialog = wx.MessageDialog(self, strError + '\n\nContinue?', 'Error', wx.YES_NO)

        if errDialog.ShowModal() == wx.ID_NO:
            self.onStop(None)

        errDialog.Destroy()


if __name__ == '__main__':
    class fakeDoc:
        def getRoot(self):
            return []


    fakeCaller = fakeDoc()
    fakeCaller.document = fakeDoc()

    app = wx.PySimpleApp()
    frame = EggbotGui(fakeCaller)
    app.MainLoop()
