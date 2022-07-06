import serial
import time

iterations = 20

ser = serial.Serial("COM6", timeout=10)

ser.write(b'V\n')
time.sleep(0.100)
s = ser.read(ser.in_waiting)
print(s.decode('utf-8'))

# Comment this block out to remove motor move from measurement
#ser.write(b'SM,10000,50000,50000\n')
#time.sleep(0.100)
#s = ser.read(ser.in_waiting)
#print(s.decode('utf-8'))

try:
    for x in range(1, iterations):
        startTime = time.perf_counter_ns()
        ser.write(b'QS,1\n')
        s = b''
        cnt = 0
        done = False
        while done == False:
            if ser.in_waiting:
                c = ser.read(1)
                s = s + c
                cnt += 1
                if c == b'\n':
                    done = True
        endTime = time.perf_counter_ns()
        # print(s.decode('utf-8'))
        tmp = ser.read(ser.in_waiting)  # Eat the rest of the reply from the EBB
        totalTimeMS = (endTime - startTime) / 1000000
        #print(str(totalTimeMS) + "  " + str(cnt))
        print(str(totalTimeMS))
finally:
    ser.close()
    print("test complete")
