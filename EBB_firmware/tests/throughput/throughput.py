import serial
import time

iterations = 1000

ser = serial.Serial('COM11', timeout=10)

#ser.write(b'VR\n')
#s = ser.readline()


startTime = time.perf_counter_ns()
for x in range(1, iterations):
    ser.write(b'VR\n')
    s = ser.readline()
    if len(s) != 28:
        print(s)
endTime = time.perf_counter_ns()
print(str(iterations) + " commands took " + str((endTime - startTime)/1000000) + "ms or " + str(((endTime - startTime)/1000000)/iterations) + "ms per command")
print("Done")
ser.close()