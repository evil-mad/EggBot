import serial
import time

ser = serial.Serial(port='COM4', baudrate=9600, timeout=0.01)

ser.isOpen()

data = ser.read()
print("Run")
#print(data)

# input values
startSpeed = 2000       # start speed in micro steps/s
endSpeed = 10000        # max speed in micro steps/s
duration = 25           # duration of entire ramp up/down in seconds
moveTime = 25          # milliseconds of each move

# computed values
stepsPerMoveStart = int(startSpeed*(moveTime/1000))
stepsIncriment = ((endSpeed - startSpeed) / (duration / (moveTime/1000))) * (moveTime/1000)
stepsPerMoveEnd = int(endSpeed*(moveTime/1000)) + int(stepsIncriment)
print(stepsPerMoveStart)
print(stepsPerMoveEnd)
print(stepsIncriment)

x = stepsPerMoveStart
y = 0
sTime = time.time()
while int(x) < stepsPerMoveEnd:
    output = b"SM," + str(moveTime).encode() + b"," + str(int(x)).encode() + b",0\r"
    print(output)
    ser.write(output)
    x = x + stepsIncriment
    y = y + 1

eTime = time.time()
print(y)
print(duration/(moveTime/1000))
print(eTime - sTime)

#data = ser.read()
#print(data)

ser.close()
exit()
