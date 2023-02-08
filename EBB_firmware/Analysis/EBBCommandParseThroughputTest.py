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

print("Test complete")

the_port.close()