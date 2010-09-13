import os, re

def findEiBotBoards():
	usbdata = os.popen( '/usr/sbin/system_profiler SPUSBDataType' ).read()
	tokens = re.split( 'EiBotBoard', usbdata )
	for t in tokens[1:]:
		match = re.match( '.*?Location ID: 0x([0-9a-fA-F]+).*', t,
				  re.M | re.S )
		if match != None:
			locid = int( match.group( 1 ), 16 )
			yield '/dev/cu.usbmodem%x' % ( ( locid >> 16 ) + 1 )

def findPorts():
	device_list = os.listdir( '/dev' )
	for device in device_list:
		if not device.startswith( 'cu.usbmodem' ):
			continue
		yield '/dev/' + device

if __name__ == '__main__':
	print "Looking for EiBotBoards"
	for port in findEiBotBoards():
		print "  ", port

	print "Looking for COM ports"
	for port in findPorts():
		print "  ", port
