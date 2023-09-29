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

command_list = [
"SM,25,94,4",
"SM,26,85,3",
"SM,25,75,3",
"SM,25,66,2",
"SM,26,57,3",
"SM,25,47,1",
"SM,25,37,2",
"SM,26,29,1",
"SM,25,19,1",
"SM,25,9,0",
"T3,364,0,494977,-1819,0,890958,-3275",
"T3,305,59652326,62088,-407,107374182,28651,-188",
"T3,305,59652326,-12675,83,107374182,60291,-395",
"T3,305,59652326,51073,-335,107374182,-46580,305",
"T3,305,59652326,-23690,155,107374182,-14940,98",
"T3,305,59652326,40058,-263,107374182,16699,-110",
"T3,305,59652326,-34705,228,107374182,48339,-317",
"T3,305,59652326,29043,-190,107374182,-58532,384",
"T3,305,59652326,-45720,300,107374182,-26893,176",
"T3,305,59652326,18028,-118,107374182,4747,-31",
"T3,305,59652326,-56735,372,107374182,36386,-239",
"T3,364,59652326,173306,-1853,107374182,253601,-3014",
"SM,26,-4,-67",
"SM,26,-3,-57",
"SM,26,-3,-48",
"SM,26,-3,-38",
"SM,26,-1,-28",
"SM,26,-1,-19",
"SM,26,-1,-10",
"T3,364,0,494977,-1819,0,890958,-3275",
"T3,305,59652326,-76423,501,107374182,-109859,720",
"T3,305,59652326,-12675,83,107374182,60291,-395",
"T3,305,59652326,51073,-335,107374182,-46580,305",
"T3,305,59652326,-23690,155,107374182,-14940,98",
"T3,305,59652326,40058,-263,107374182,16699,-110",
"T3,305,59652326,-34705,228,107374182,48339,-317",
"T3,305,59652326,29043,-190,107374182,-58532,384",
"T3,305,59652326,-45720,300,107374182,-26893,176",
"T3,305,59652326,18028,-118,107374182,4747,-31",
"T3,305,59652326,-56735,372,107374182,36386,-239",
"T3,364,59652326,173306,-1853,107374182,350849,-3549",
"SM,26,51,-55",
"SM,27,101,-110",
"SM,26,152,-165",
"SM,26,203,-221",
"SM,27,254,-275",
"SM,156,1806,-1960",
"SM,27,254,-275",
"SM,26,203,-221",
"SM,26,152,-165",
"SM,27,101,-110",
"SM,26,51,-55",
"T3,364,0,494977,-1819,0,890958,-3275",
"T3,305,59652340,-76422,501,107374182,28651,-188",
"T3,305,59652340,-12675,83,107374182,-78220,513",
"T3,305,59652340,51073,-335,107374182,91930,-603",
"T3,305,59652340,-23689,155,107374182,-14941,98",
"T3,305,59652340,40058,-263,107374182,-121812,799",
"T3,305,59652340,-34704,228,107374182,48337,-317",
"T3,305,59652340,29044,-190,107374182,-58533,384",
"T3,305,59652340,-45719,300,107374182,111616,-732",
"T3,305,59652340,18029,-118,107374182,4745,-31",
"T3,305,59652340,-56733,372,107374182,-102126,670",
"T3,364,59652340,173307,-1853,107374182,350847,-3549",
"SM,26,-58,47",
"SM,27,-117,93",
"SM,26,-176,141",
"SM,26,-233,187",
"SM,27,-292,234",
"SM,179,-2380,1906",
"SM,26,-292,234",
"SM,26,-233,187",
"SM,27,-176,141",
"SM,26,-117,93",
"SM,26,-58,47",
]



response = ebb_serial.query(the_port, 'V\r')
print("Firmware version: " + response)

# response = ebb_serial.query(the_port, "QU,2\r") # EBB 3+: Read max FIFO depth
# print("FIFO Max depth query: " + response.strip())

# ebb_serial.command(the_port, "CU,3,1" + '\r') # Enable data-low LED
# ebb_serial.command(the_port, "CU,4,25" + '\r') # Set FIFO depth

# response = ebb_serial.query(the_port, "QU,3\r") # EBB 3+: Read new FIFO depth
# print("FIFO new depth query: " + response)


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
response = ebb_serial.query(the_port, 'QU,4\r')
print("Final QU,4:: " + response.strip())

# response = ebb_serial.query(the_port, "QU,4\r") # EBB 3+: Read EBB stack max depth
# print("Max stack depth: " + response)

# print(last_command + " :: ")


ad.disconnect()             # Close serial port to AxiDraw
