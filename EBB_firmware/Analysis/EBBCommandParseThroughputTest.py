# A standardized way to test the throughput (how many commands per second) of an EBB
# This test can be used to measure the relative changes in command parsing throughput
# as the EBB firmware is changed.
# It can also provide an rough absolute metric of how many USB commands per second an EBB 
# can process.
# 
# The basic idea is to send various commands to the EBB, and measure the time it takes for
# the EBB to process all of them, then print out the average time/command.
#
# The most basic command is something like the "NI" (Node Increment) command, which is 
# not a motion command, executes very quickly since it only modifies one internal 
# variable, and returns very little (just an "OK"). This will be the fastest command.
#
# This command (NI) can be tested during various conditions:
# * When no motion or other commands are happening
# * When simple SM/XM/HM commands are ongoing (i.e. both steppers are moving)
# * When complex motion commands are ongoing (LM/LT)
# Comparing the values for these times can give an idea of how much the various types
# of motion commands load down the USB processing
#
# After sending the command, we wait for the full response to come back from the EBB
# before continuing with the next command. This is the worst-case timing use case.
#
# The other test is to use a simple SM command which is very short, and repeat it
# over and over to see how fast motion commands can be processed.
#
# Each command is sent one a a time, rather than multiple commands packet together,
# to mimic the real life usage of the EBB.

import sys
import time
from plotink import ebb_serial

def query(port_name, cmd):
    ''' Simple serial query function '''
    port_response = ''
    if port_name is not None and cmd is not None:
        try:
            port_name.write(cmd.encode('ascii'))
            port_response = port_name.readline()
            n_retry_count = 0
            while len(port_response) == 0 and n_retry_count < 100:
                port_response = port_name.readline()
                n_retry_count += 1
        except:
            print("Error reading serial data.")
    return port_response

the_port = ebb_serial.openPort()
if the_port is None:
    print("failed to connect")
    sys.exit() # end script

the_port.reset_input_buffer()

print("Starting EBB USB throughput testing.")
response = query(the_port, 'V\r')
print(response)

"""

# Test 10000 "NI" commands with no motion
startTime = time.perf_counter()
for x in range(10000):
    query(the_port, "NI" + '\r')
endTime = time.perf_counter()

print(f"{((endTime - startTime)*1000)/10000:3.3f} ms per NI command when motion idle")

time.sleep(5)

# Test 10000 "NI" commands with simple motion
query(the_port, "SM,10000,200000,200000" + '\r')

startTime = time.perf_counter()
for x in range(10000):
    query(the_port, "NI" + '\r')
endTime = time.perf_counter()

print(f"{((endTime - startTime)*1000)/10000:3.3f} ms per NI command with simple (SM/XM/HM) motion")

time.sleep(10)

# Test 10000 "NI" commands with complex motion
query(the_port, "LT,250000,858999350,-5436,858999350,-5436,3" + '\r')

startTime = time.perf_counter()
for x in range(10000):
    query(the_port, "NI" + '\r')
endTime = time.perf_counter()

print(f"{((endTime - startTime)*1000)/10000:3.3f} ms per NI command with complex (LM/LT) motion")

time.sleep(15)

"""

# Section to test fastest separate move commands tolerated by EBB Firmware
# Note that worst case timings are used here - max step rate (25Khz) and both steppers stepping
# together at that rate. This gives the CPU the least amount of time to parse USB commands as 
# possible.
#
# When this code is run, it should be used with the GPIO_DEBUG and UART_OUTPUT_DEBUG defined so that 
# it is easy to see on the logic analyzer when the FIFO is not full all the time. The UART output
# can then be used to see which command caused the un-full FIFO.

# First for SM command
"""
for x in range(100):
    query(the_port, "SM,10,250,250" + '\r')
for x in range(100):
    query(the_port, "SM,9,225,225" + '\r')
for x in range(100):
    query(the_port, "SM,8,200,200" + '\r')
for x in range(100):
    query(the_port, "SM,7,175,175" + '\r')
for x in range(100):
    query(the_port, "SM,6,150,150" + '\r')
for x in range(100):
    query(the_port, "SM,5,125,125" + '\r')
for x in range(100):
    query(the_port, "SM,4,100,100" + '\r')
for x in range(100):
    query(the_port, "SM,3,75,75" + '\r')
for x in range(100):
    query(the_port, "SM,2,50,50" + '\r')
for x in range(100):
    query(the_port, "SM,1,25,25" + '\r')

time.sleep(0.5)

# Then for XM command
for x in range(100):
    query(the_port, "XM,10,250,0" + '\r')
for x in range(100):
    query(the_port, "XM,9,225,0" + '\r')
for x in range(100):
    query(the_port, "XM,8,200,0" + '\r')
for x in range(100):
    query(the_port, "XM,7,175,0" + '\r')
for x in range(100):
    query(the_port, "XM,6,150,0" + '\r')
for x in range(100):
    query(the_port, "XM,5,125,0" + '\r')
for x in range(100):
    query(the_port, "XM,4,100,0" + '\r')
for x in range(100):
    query(the_port, "XM,3,75,0" + '\r')
for x in range(100):
    query(the_port, "XM,2,50,0" + '\r')
for x in range(100):
    query(the_port, "XM,1,25,0" + '\r')

time.sleep(0.5)

# Then for LM command
for x in range(100):
    query(the_port, "LT,250,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,225,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,200,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,175,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,150,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,125,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,100,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,75,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,50,2147483647,-1,2147483647,-1" + '\r')
for x in range(100):
    query(the_port, "LT,25,2147483647,-1,2147483647,-1" + '\r')

time.sleep(0.5)
"""

# Test LM command- specifically looking for step position errors 
# Clear the step position at the beginning, and then check it after
# each test.
#
# First just straight non-accelerating moves in a square
query(the_port, "CS" + '\r')
for x in range(10):
    query(the_port, "LM,85899350,100,0,85899350,100,0" + '\r')
for x in range(10):
    query(the_port, "LM,85899350,-100,0,85899350,100,0" + '\r')
for x in range(10):
    query(the_port, "LM,85899350,-100,0,85899350,-100,0" + '\r')
for x in range(10):
    query(the_port, "LM,85899350,100,0,85899350,-100,0" + '\r')

time.sleep(1.0)

response = query(the_port, 'QS\r')
print(response)

# Now each set of moves is an accel down to zero and then back up again as separate commands
for x in range(10):
    query(the_port, "LM,85899350,100,-34360,85899350,100,-34360" + '\r')
    query(the_port, "LM,0,100,34360,0,100,34360" + '\r')
for x in range(10):
    query(the_port, "LM,85899350,-100,-34360,85899350,100,-34360" + '\r')
    query(the_port, "LM,0,-100,34360,0,100,34360" + '\r')
for x in range(10):
    query(the_port, "LM,85899350,-100,-34360,85899350,-100,-34360" + '\r')
    query(the_port, "LM,0,-100,34360,0,100,34360" + '\r')
for x in range(10):
    query(the_port, "LM,85899350,100,-34360,85899350,-100,-34360" + '\r')
    query(the_port, "LM,0,100,34360,0,100,34360" + '\r')

time.sleep(1.0)

response = query(the_port, 'QS\r')
print(response)

print("Test complete")

the_port.close()