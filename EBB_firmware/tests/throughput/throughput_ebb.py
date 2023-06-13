# Compare 'normal' USB command processing speed when not stepping and when stepping
# Commands are sent one at a time, not bunched up together
import serial
import time

iterations = 1000
commandsPerIteration = 1

ser = serial.Serial("COM6", timeout=10)

#ser.write(b'VR\n')
s = ser.read(ser.in_waiting)

try:
# We first run a whole bunch of "v" commands without stepping
    startTimeNoStep = time.perf_counter_ns()
    for x in range(1, iterations):
        ser.write(b'v\n')
        s = ser.read(ser.in_waiting)
#        print(s.decode('utf-8'))
    endTimeNoStep = time.perf_counter_ns()

# Then we run the same number of "v" commands with 25KHz stepping
    ser.write(b'SM,10000,250000,250000\n')
    startTimeStep = time.perf_counter_ns()
    for x in range(1, iterations):
        ser.write(b'v\n')
        s = ser.read(ser.in_waiting)
#        print(s.decode('utf-8'))
    endTimeStep = time.perf_counter_ns()

    print("When not stepping : " + str(((endTimeNoStep - startTimeNoStep)/1000000)/(iterations * commandsPerIteration)) + "ms per command")
    print("When stepping     : " + str(((endTimeStep - startTimeStep)/1000000)/(iterations * commandsPerIteration)) + "ms per command")
    print("Done")
finally:
    ser.close()
    print("Closed port")