import os
import re

DEV_TREE = '/dev'
USB_DEV_PREFIX = 'cu.usbmodem'

def findEiBotBoards():
	usbdata = os.popen( '/usr/sbin/system_profiler SPUSBDataType' ).read()
	tokens = re.split( 'EiBotBoard', usbdata )
	for t in tokens[1:]:
		match = re.match( '.*?Location ID: 0x([0-9a-fA-F]+).*', t, re.M | re.S )
		if match != None:
			locid = int( match.group( 1 ), 16 )
			yield os.path.join( DEV_TREE,
					    '%s%x' % ( USB_DEV_PREFIX, ( ( locid >> 16 ) + 1 ) ) )

def findPorts():
	device_list = os.listdir( DEV_TREE )
	for device in device_list:
		if not device.startswith( USB_DEV_PREFIX ):
			continue
		yield os.path.join( DEV_TREE, device )

if __name__ == '__main__':
	print "Looking for EiBotBoards"
	for port in findEiBotBoards():
		print "  ", port

	print "Looking for COM ports"
	for port in findPorts():
		print "  ", port
