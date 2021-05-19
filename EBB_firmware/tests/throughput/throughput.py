import serial
import time

iterations = 100

ser = serial.Serial("COM6", timeout=10)
#ser = serial.Serial("COM7", timeout=10)

#ser.write(b'VR\n')
s = ser.read(ser.in_waiting)

try:
    startTime = time.perf_counter_ns()
    for x in range(1, iterations):
#        ser.write(b'VR\n')
#        ser.write(b'VR\nVR\nVR\nVR\nVR\nVR\nVR\nVR\nVR\nVR\n')
        ser.write(b'SM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\nSM,5,1,1,1\n')
#        ser.write(b'V\nV\nV\nV\nV\nV\nV\nV\nV\nV\n')
#        ser.write(b'VR\n')
        s = ser.read(ser.in_waiting)
#        if len(s) != 27:       # for 3BB
#        if len(s) != 44:        # for EBB
        print(s.decode('utf-8'))
    endTime = time.perf_counter_ns()
    print(str(iterations) + " commands took " + str((endTime - startTime)/1000000) + "ms or " + str(((endTime - startTime)/1000000)/iterations) + "ms per command")
    print("Done")
finally:
    ser.close()
    print("Closed port")