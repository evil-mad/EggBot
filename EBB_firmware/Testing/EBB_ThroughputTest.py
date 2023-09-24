#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
Script to send a command list to EBB

'''



import sys
import time
import copy

from pyaxidraw import axidraw
from plotink import ebb_motion
from plotink import ebb_serial
# print("hello")

def query(port_name, cmd):
    if port_name is not None and cmd is not None:
        response = ''
        try:
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


def cmd_mod(port_name, cmd, verbose=True):
    '''General command to send a command to the EiBotBoard'''
    if port_name is not None and cmd is not None:
        port_name.write(cmd.encode('ascii'))
        response = port_name.readline().decode('ascii')
        n_retry_count = 0
        while len(response) == 0 and n_retry_count < 100:
            # get new response to replace null response if necessary
            response = port_name.readline().decode('ascii')
            n_retry_count += 1
        if response.strip().startswith("OK"):
            pass
        else:
            if response:
                error_msg = '\n'.join(('Unexpected response from EBB.',
                                       '    Command: {0}'.format(cmd.strip()),
                                       '    Response: {0}'.format(response.strip())))
            else:
                error_msg = 'EBB Serial Timeout after command: {0}'.format(cmd)
            if verbose:
                print(error_msg)
            else:
                print(error_msg)





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

command_list_new = [
"SM,30,100,0",
#"SM,30,200,0",
#"SM,30,-200,0",
"SM,30,-100,0",
]

command_list = [
"SM,30,111,-34",
"SM,30,221,-69",
"SM,30,331,-102",
"SM,30,443,-137",
"SM,30,331,-103",
"SM,30,221,-69",
"SM,29,111,-34",
"SM,11,16,-13",
"SM,7,17,-4",
"SM,6,17,4",
"SM,6,18,13",
"SM,5,13,18",
"SM,4,4,17",
"SM,4,-5,16",
"SM,4,-12,17",
"SM,27,-89,89",
"SM,27,-72,72",
"SM,27,-55,54",
"SM,26,-38,38",
"SM,28,78,78",
"SM,28,89,89",
"SM,4,11,14",
"SM,4,5,17",
"SM,4,-3,18",
"SM,5,-13,18",
"SM,5,-18,13",
"SM,4,-18,3",
"SM,4,-17,-5",
"SM,4,-15,-11",
"SM,36,-101,-101",
"SM,36,-65,-65",
"SM,28,-42,41",
"SM,28,-35,35",
"SM,28,-28,28",
"SM,37,111,111",
"SM,38,122,122",
"SM,4,11,15",
"SM,4,5,16",
"SM,4,-3,18",
"SM,5,-13,19",
"SM,4,-18,12",
"SM,4,-18,3",
"SM,4,-17,-4",
"SM,4,-14,-11",
"SM,30,-107,-107",
"SM,30,-85,-85",
"SM,29,-65,-65",
"SM,30,-44,-44",
"SM,26,65,-65",
"SM,26,104,-104",
"SM,26,143,-143",
"SM,30,113,-113",
"SM,30,61,-61",
"SM,30,7,-7",
"SM,30,-111,34",
"SM,30,-221,69",
"SM,30,-331,102",
"SM,30,-443,137",
"SM,30,-331,103",
"SM,30,-221,69",
"SM,29,-111,34",
]


#response = ebb_serial.command(the_port, "CU,250,1\r")
#print(response)
#response = ebb_serial.command(the_port, "CU,253,1\r")
#print(response)

response = ebb_serial.query(the_port, 'V\r')
print("Firmware version: " + response)

response = ebb_serial.query(the_port, "QU,2\r") # EBB 3+: Read max FIFO depth
print("FIFO Max depth query: " + response.strip())

ebb_serial.command(the_port, "CU,3,1" + '\r') # Enable data-low LED
ebb_serial.command(the_port, "CU,4,25" + '\r') # Set FIFO depth

response = ebb_serial.query(the_port, "QU,3\r") # EBB 3+: Read new FIFO depth
print("FIFO new depth query: " + response)

last_command = ""

for command in command_list:

    cmd_type = command.split(",")[0].strip().lower()
    # print("command: " + command + ", cmd_type: " + cmd_type)

    if cmd_type in ["qs"]:
        # response = str(query(the_port, command + '\r'))
        response = ebb_serial.query(the_port, command + '\r')
        print(last_command + " :: " + response.strip())
    else:
        cmd_mod(the_port, command + '\r')
        # ebb_serial.command(the_port, command + '\r')
        # print(last_command + " :: " )
    last_command = command

block(ad)
response = ebb_serial.query(the_port, 'QS\r')
print("Final QS:: " + response.strip())

#response = ebb_serial.query(the_port, "QU,4\r") # EBB 3+: Read EBB stack max depth
#print("Max stack depth: " + response)

# print(last_command + " :: ")


ad.disconnect()             # Close serial port to AxiDraw
