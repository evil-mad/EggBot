import sys

platform = sys.platform.lower()

if platform == 'win32':
	from eggbot_scanwin32 import *
elif platform == 'darwin':
	from eggbot_scanosx import *
elif platform == 'linux2':
	from eggbot_scanlinux import *
else:
	from eggbot_scanposix import *
