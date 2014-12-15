import _winreg
import re

def findEiBotBoards():
	hReg = _winreg.ConnectRegistry( None, _winreg.HKEY_LOCAL_MACHINE )
	hKey = _winreg.OpenKey( hReg, r"SYSTEM\CurrentControlSet\Enum\USB\VID_04D8&PID_FD92" )
	nKeys,nVals,nTime = _winreg.QueryInfoKey( hKey )
	for i in range( nKeys ):
		dev = _winreg.EnumKey( hKey, i )
		hKey2 = _winreg.OpenKey( hKey, dev )
		try:
			fname,t = _winreg.QueryValueEx( hKey2, "FriendlyName" )
			match = re.search( r".*\((.*)\)$", fname )
			yield match.group( 1 )
		except GeneratorExit:
			_winreg.CloseKey( hKey )
			_winreg.CloseKey( hReg )
			raise StopIteration
		except:
			pass
		finally:
			_winreg.CloseKey( hKey2 )

	# The next two lines may not executed when our caller
	# succeeds in finding an Eggbot device: in that case
	# the caller may do a "break" which then triggers the
	# "except GeneratorExit" clause above.
	_winreg.CloseKey( hKey )
	_winreg.CloseKey( hReg )

def findPorts():
	found = 0
	hReg = _winreg.ConnectRegistry( None, _winreg.HKEY_LOCAL_MACHINE )
	hKey = _winreg.OpenKey( hReg, r"SOFTWARE\Microsoft\Windows NT\CurrentVersion\Ports" )
	nKeys,nVals,nTime = _winreg.QueryInfoKey( hKey )
	for i in range( nVals ):
		n,v,t = _winreg.EnumValue( hKey, i )
		if n[0:3] == 'COM':
			found = 1
			try:
				if n[-1] == ':':
					yield n[:-1]
				else:
					yield n
			except GeneratorExit:
				_winreg.CloseKey( hKey )
				_winreg.CloseKey( hReg )
				raise StopIteration

	# The next two lines may not executed when our caller
	# succeeds in finding an Eggbot device: in that case
	# the caller may do a "break" which then triggers the
	# "except GeneratorExit" clause above.
	_winreg.CloseKey( hKey )
	_winreg.CloseKey( hReg )

	# If we didn't find anything, then produce COM1, COM2, COM3, ..., COM99
	if found == 0:
		for i in range( 1, 100 ):
			yield "COM" + str( i )

if __name__ == '__main__':
	print "Looking for EiBotBoards"
	for port in findEiBotBoards():
		print "  ", port

	print "Looking for COM ports"
	for port in findPorts():
		print "  ", port
