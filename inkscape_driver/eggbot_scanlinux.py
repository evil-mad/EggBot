import os

DEV_TREE = '/dev'
USB_DEVICE_TREE = '/sys/bus/usb/devices'

def findEiBotBoards():
	"""Find only those USB devices that declare themselves to be EiBotBoards"""

	# find all USB devices whose product name is 'EiBotBoard'
	with os.popen( 'fgrep -l EiBotBoard %s/*/product' % USB_DEVICE_TREE ) as pipe:
		for path in [ os.path.split( path )[0] for path in pipe.readlines()]:
			device = os.path.split( path )[1]

			# for each device endpoint ...
			for dir in os.listdir( path ):
				if dir.startswith( device ):

					# find the endpoint that supports tty access
					ttydir = os.path.join( USB_DEVICE_TREE, device, dir, 'tty' )
					if os.path.exists( ttydir ):

						# And emit each (the) interface name
						for ttyname in os.listdir( ttydir ):
							yield os.path.join( DEV_TREE, ttyname )

def findPorts():
	for device in os.listdir( DEV_TREE ):
		if not device.startswith( 'ttyACM' ):
			continue
		yield os.path.join( DEV_TREE , device )

if __name__ == '__main__':
	print "Looking for EiBotBoards"
	for port in findEiBotBoards():
		print "  ", port

	print "Looking for COM ports"
	for port in findPorts():
		print "  ", port
