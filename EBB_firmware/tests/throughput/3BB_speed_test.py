# Ramp up from an inital speed to a maximum speed in mulitple steps with the goal
# of finding the maximum sustainable speed for a given configuration. Run on 
# EBB and 3BB to compare max speeds.
import serial
import time
from ast import literal_eval

minSpeed =     10000   # in microsteps/second - all runs start at this speed
maxSpeed1 =    14000   # in microsteps/second - first run maxes at this speed
maxSpeed2 =    18000   # in microsteps/second - last run maxes at this speed
bigIterations =   20   # Number of outer loop up/down speed attempts
iterations =      10   # Number of inner loop move segments

# Send a motion control string to the EBB or 3BB
# First check to make sure that there is room in the FIFO
def sendMotion(sendString):
    done = False
    while done == False:
        mFIFO = 0
        CMD = 0
        # Suck out any existing reply from xBB (should be nothing)
        s = ser.read(ser.in_waiting)
        if (len(s) > 0):
            print(s.decode('utf-8'))
        # Send the QG command to get data about the FIFO status
        ser.write("QG\n".encode("utf-8"))
        status = ser.read_until(b'\n')
        statusStr = "0x" + status.decode("utf-8")
        #print(statusStr)
        hexVal = literal_eval(statusStr)
        if (hexVal & 8) > 0:
            CMD = 1
        if (hexVal & 1) > 0:
            mFIFO = 1
        if mFIFO == 0:
            done = True

    ser.write(sendString.encode("utf-8"))


#ser = serial.Serial("COM5", timeout=1)
ser = serial.Serial("COM4", timeout=1)

ser.write(b'V\n')
time.sleep(0.100)
s = ser.read(ser.in_waiting)
print(s.decode('utf-8'))

# Turn off "OK" responses since we don't care about those
ser.write(b'CU,1,0\n')
time.sleep(0.100)
s = ser.read(ser.in_waiting)
print(s.decode('utf-8'))

# Comment this block out to remove motor move from measurement
#ser.write(b'SM,10000,50000,50000\n')
#time.sleep(0.100)
#s = ser.read(ser.in_waiting)
#print(s.decode('utf-8'))

try:
    for endSpeed in range(maxSpeed1,maxSpeed2 + int((maxSpeed2 - maxSpeed1)/bigIterations),int((maxSpeed2 - maxSpeed1)/bigIterations)):
        # Send the commands to ramp from off to endSpeed
        duration = 100
        for speed in range(minSpeed, endSpeed + int((endSpeed - minSpeed)/iterations), int((endSpeed - minSpeed)/iterations)):
            steps = speed*(duration/1000)
            printStr = "SM," + str(duration) + "," + str(int(steps)) + "\n"
            sendMotion(printStr)
            print("speed = " + str(speed))
        print("top " + str(speed))
        # Now set at endSpeed for a while
        for x in range(5):
            sendMotion(printStr)
        # Then ramp back down to off
        for speed in range(endSpeed, minSpeed + int((endSpeed - minSpeed)/iterations), -(int((endSpeed - minSpeed)/iterations))):
            steps = speed*(duration/1000)
            printStr = "SM," + str(duration) + "," + str(int(steps)) + "\n"
            sendMotion(printStr)
        #time.sleep(0.5)
        s = ser.read(ser.in_waiting)
        if (len(s) > 0):
            print(s.decode('utf-8'))

finally:
    ser.close()
    print("test complete")
