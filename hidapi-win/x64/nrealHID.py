import os, ctypes
ctypes.CDLL('./hidapi.dll')

import hid
print("test")

vid = 0x3318	# Change it for your device
pid = 0x0424	# Change it for your device
device = None

devs = hid.enumerate(vid, pid)
for dev in devs:
	print(dev)
	print("")
	if (dev["interface_number"] == 3):
		print("Found interface 3")
		
		break

dev = hid.Device(vid, pid)
dev.nonblocking = 1
dev.write(b'\xaa, \xc5, \xd1, \x21, \x42, \x04, \x00, \x19, \x01')
while True:
	res = dev.read(64)
	print(res)
	

