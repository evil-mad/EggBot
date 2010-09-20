import os
import sys

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
		if strPrefix:
			if not device.startswith( strPrefix ):
				continue
			yield os.path.join( strDir, device )
