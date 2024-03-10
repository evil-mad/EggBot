#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
python_send_cmds3.py

Run this demo by calling: python3 python_send_cmds3.py

'''



import sys
import time

from pyaxidraw import axidraw
from plotink import ebb_motion
from plotink import ebb_serial
# print("hello")

def query(port_name, cmd):
    if port_name is not None and cmd is not None:
        response = ''
        try:
            port_name.timeout = 0.1
            port_name.write(cmd.encode('ascii'))
            response = port_name.readline()
            n_retry_count = 0
            while len(response) == 0 and n_retry_count < 1:
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

# At first this will just send the command.

# Eventually this code can capture and analyze the result from the Saleae to generate
# the answers fully automatically.

SM_command_list = [
#"SM,15,375,375",       # 25K
#"SM,15,37,37",       # 2.5K
#"SM,14,350,350",
#"SM,13,325,325",
#"SM,12,300,300",
#"SM,11,275,275",
#"SM,10,250,250",
#"SM,9,225,225",
#"SM,8,200,200",
#"SM,7,175,175",        # 25K
"SM,7,17,17",         # 2.5K
#"SM,6,150,150",
#"SM,5,125,125",
#"SM,4,100,100",
#"SM,3,75,75",
#"SM,2,50,50",
#"SM,1,25,25",
]

LM_command_list = [
"LM,2147483647,250,-1,2147483647,250",
"LM,2147483647,225,-1,2147483647,225",
"LM,2147483647,200,-1,2147483647,200",
"LM,2147483647,175,-1,2147483647,175",
"LM,2147483647,150,-1,2147483647,150",
"LM,2147483647,125,-1,2147483647,125",
"LM,2147483647,100,-1,2147483647,100",
"LM,2147483647,75,-1,2147483647,75",
"LM,2147483647,50,-1,2147483647,50",
"LM,2147483647,25,-1,2147483647,25",
]

# CU,4 sets new FIFO length
# QU,2 reads back max FIFO length

print("connected")

response = query(the_port, 'V\r')
print(response)
# response = query(the_port, 'CU,250,1\r')    # Turn on ISR GPIO debug bits
# print(response)
#response = query(the_port, 'CU,1,0\r')      # Turn off 'ack'/OK
#print(response)
response = query(the_port, 'CU,2,0\r')      # Turn off limit checking
print(response)
# response = query(the_port, 'CU,10,1\r')     # Turn on standardized command format replies
# print(response)

last_command = ""

for command in SM_command_list:
    #block(ad)
    for i in range(100):
        response = str(query(the_port, command + '\r'))
        print(last_command + " :: " + response.strip())
        last_command = command
    
    time.sleep(.25)

print(last_command + " :: ")

time.sleep(1.0)

#for command in LM_command_list:
#    #block(ad)
#    for i in range(100):
#        response = str(query(the_port, command + '\r'))
#        print(last_command + " :: " + response.strip())
#        last_command = command
    
#    time.sleep(.25)

#print(last_command + " :: ")

time.sleep(1.0)


ad.disconnect()             # Close serial port to AxiDraw

print("Test complete")









