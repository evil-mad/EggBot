#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
python_send_cmds3.py

Run this demo by calling: python3 python_send_cmds3.py

'''



import sys
import time
import random

from pyaxidraw import axidraw
from plotink import ebb_motion
from plotink import ebb_serial
# print("hello")

# Just send the command, don't read a response
def EBB_send(port_name, cmd):
    if port_name is not None and cmd is not None:
        response = ''
        try:
            port_name.write(cmd.encode('ascii'))
        except:
            print("Error reading serial data.")


def query(port_name, cmd):
    if port_name is not None and cmd is not None:
        response = ''
        try:
#             port_name.write(cmd.encode('ascii'))
            port_name.write(cmd.encode('ascii'))
            response = port_name.readline()
            n_retry_count = 0
            while len(response) == 0 and n_retry_count < 100:
                # get new response to replace null response if necessary
                response = port_name.readline()
                n_retry_count += 1
        except:
            print("Error reading serial data.")
        return response

def block(ad_ref, timeout_ms=None):
    '''
    Interactive context: Wait until all motion control commands have finished, or an
    an optional timeout occurs.

    Polls the EBB immediately and then every 10 ms thereafter until (1) Neither motor is
    currently in motion and (2) there is no queued motion control command.

    A value for timeout_ms, gives the maximum duration to wait in milliseconds.

    Returns True if the motion queue is empty, and False if timed out.

    Requires EBB version v2.6.2 or newer
    '''

    if timeout_ms is None:
        time_left = 60000 # Default 60 second timeout.
    else:
        time_left = timeout_ms

    while True:
        qg_val = bytes.fromhex(ebb_serial.query(ad_ref.plot_status.port, 'QG\r').strip())
        motion = qg_val[0] & (15).to_bytes(1, byteorder='big')[0] # Motion queue bits
        if motion == 0:
            return True
        if time_left <= 0:
            return False    # Timed out
        if time_left < 10:
            time.sleep(time_left / 1000) # Use up remaining time
            time_left = 0
        else:
            time.sleep(0.01) # Sleep 10 ms
            if timeout_ms is not None:
                time_left -= 10


ad = axidraw.AxiDraw() # Initialize class

ad.interactive()

if not ad.connect():                # Open serial port to AxiDraw;
    print("failed to connect")
    quit()   

the_port = ad.plot_status.port
if the_port is None:
    print("failed to connect")
    sys.exit() # end script

the_port.reset_input_buffer()


# This is for v3.0.0 EBB firmwares and above (i.e. ones that have the L3 command and
# the ability to change it's FIFO depth)

# This script will create random sets of input parameters to the SM command.
# It is meant to be used with the version of v3.0.0 (like -a34) which does not actually
# put the new command onto the FIFO. Rather, we use CU,255,1 so that the output of the
# SM math gets printed back to the PC. This scrip then computes the correct values for
# the two rates, and compares what the EBB computed, recording everything (including the
# error percentage) to a log file.
#
# The idea is to let this thing run for hours at a time, randomly trying various inputs
# and printing out an alert any time an error threshold is exceeded.

print("connected, test beginning")

response = query(the_port, 'V\r')
if len(response.strip()) > 0:
    print(response.strip())

response = query(the_port, 'CU,255,1\r')    # Turn on the printing of SM final rate values
if len(response.strip()) > 0:
    print(response.strip())

response = query(the_port, 'CU,1,0\r')      # Turn off ack
if len(response.strip()) > 0:
    print(response.strip())

last_command = ""

randDuration = 0
randStep1 = 0
randStep2 = 0
idealRate1 = 0
idealRate2 = 0
outputRate1 = 0
outputRate2 = 0

outFile = open("SMRateCheckOutput.csv", "w")

#for i in range(1694117):        # 8hrs
#for i in range(10):
for i in range(211764):          # 1hr
    # Create a set of input parameters

    # AxiDraw SE/A3 has drawing size of 430 mm x 297 mm
    # EBB gives us about 160 steps/mm, so our max move length
    # will be 430 * 160 = 68800 steps

    # This set is the full set that SM can take in
    randDuration = int(random.randrange(0, 4294967295))
    randStep1 = int(random.randrange(-2147483647,2147483647))
    randStep2 = int(random.randrange(-2147483647,2147483647))

    # This set is the typical set used in AxiDraw
    randDuration = int(random.randrange(0, 65535))
    randStep1 = int(random.randrange(-100000,100000))
    randStep2 = int(random.randrange(-100000,100000))

    # Check to see if this combination of random parameters would cause a step rate 
    # above 25 kHz, or other not-allowed combinations of values
    if randDuration == 0:
        continue
    if randStep1 == 0 and randStep2 == 0:
        continue
    if (abs(randStep1)/(randDuration/1000) > 25000):
        continue
    if (abs(randStep2)/(randDuration/1000) > 25000):
        continue

    # Put them into a command and send to the EBB
    command = "SM," + str(randDuration) + "," + str(randStep1) + "," + str(randStep2)
    EBB_send(the_port, command + '\r')

    # Keep pulling in full lines until you get one with an error or one that starts with R1=
    response = ""
    done = False
    retries = 0
    outputRate2 = 0
    outputRate1 = 0
    while done == False: 
        response = the_port.readline().strip()
        respStr = response.decode("utf-8")

        if len(response) == 0:
            retries = retries + 1

        if response[0:1] == b'R':
            # Extract the two rates from the response
            respList = respStr.split()
            outputRate1 = int(respList[0][3:])
            outputRate2 = int(respList[2][3:])
            done = True

        if response[0:1] == b'!':
            # An error has occurred
            print('got an error : ' + command + " : " + respStr)
            done = True

        if retries > 4:
            print('timeout reading')
            done = True

    # if we got good data
    if outputRate1 != 0 and outputRate2 != 0:
        # Compute the proper values for the two rates
        idealRate1 = abs(int((float(randStep1)/(float(randDuration)/1000.0))*85899.34592))
        idealRate2 = abs(int((float(randStep2)/(float(randDuration)/1000.0))*85899.34592))

        # Compare them with the ideal and print the result
        diffRate1 = idealRate1 - outputRate1
        diffRate2 = idealRate2 - outputRate2
        diffPercent1 = (diffRate1/idealRate1) * 100
        diffPercent2 = (diffRate2/idealRate2) * 100

        # Generate an output string for this command and analysis
        finalOutStr = command + ',' + '%d' % idealRate1 + ',' + '%d' % outputRate1 + ',' + '%.8f' % diffPercent1 + ',' + '%d' % idealRate2 + ',' + '%d' % outputRate2 + ',' + '%.8f' % diffPercent2

        # Write every line out to the file
        outFile.write(finalOutStr + '\n')

        if diffPercent1 > 0.00001 or diffPercent2 > 0.00001:
            print(finalOutStr)

        #print(command + " :: " + str(response.strip()))
    
print("Test complete.")
response = the_port.readline()
n_retry_count = 0
while len(response) != 0:
    response = the_port.readline()
    if len(response) > 0:
        print(response)

ad.disconnect()             # Close serial port to AxiDraw
