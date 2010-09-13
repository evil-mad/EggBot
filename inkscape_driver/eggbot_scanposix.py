import os, sys

platform = sys.platform.lower()

if platform == 'darwin':
    strDir = '/dev'
    strPrefix = 'cu.usbmodem'
elif platform == 'sunos5':
    strDir = '/dev/term'
    strPrefix = None
else:
    strDir = '/dev'
    strPrefix = 'ttyACM'

def findEiBotBoards():
    return []

def findPorts():
    for device in os.listdir( strDir ):
        if strPrefix != None:
            if not device.startswith( strPrefix ):
                continue
            yield strDir + '/' + device
