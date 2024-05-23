#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
Script to send a command list to EBB

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


# Motor 1 only
command_list = [
"CU,250,1",
"CU,251,1",
"T3,22,490123456,0,0,0,0,0",
"T3,22,-490123456,0,0,0,0,0",
"T3,2,1073741824,0,0,0,0,0",
"T3,2,-1073741824,0,0,0,0,0",
"T3,100,0,0,400000,0,0,0",
"T3,100,0,0,-400000,0,0,0",
"T3,19512,0,0,11,0,0,0",
"T3,19512,0,0,-11,0,0,0",
"T3,19512,-490123456,0,11,0,0,0",
"T3,19512,490123456,0,-11,0,0,0",
"T3,30000,0,-95000,11,0,0,0",
"T3,30000,0,95000,-11,0,0,0",
"T3,35000,-490123456,-125000,11,0,0,0",
"T3,35000,490123456,125000,-11,0,0,0",
"T3,35000,-490123456,-125000,11,0,0,0,2047483648",
"T3,35000,490123456,125000,-11,0,0,0,2047483648",
"T3,20,490123456,0,0,0,0,0,1073741823",
"T3,20,-490123456,0,0,0,0,0,1073741823",
"T3,85,8589934,17353403,0,0,0,0",
"T3,85,-8589934,-17353403,0,0,0,0",
"T3,83,8589934,17353403,0,0,0,0,2047483648",
"T3,83,-8589934,17353403,0,0,0,0,2047483648",
"T3,22,490123456,0,100000,0,0,0",
"T3,22,-490123456,0,100000,0,0,0",
"T3,22,490123456,0,-100000,0,0,0",
"T3,22,-490123456,0,-100000,0,0,0",
"T3,69,1800095000,-26012345,600999,0,0,0",
"T3,69,-1800095000,26012345,-600999,0,0,0",
"T3,30,0,50353403,0,0,0,0",
"T3,30,0,-50353403,0,0,0,0",
"T3,30,0,50353403,100,0,0,0",
"T3,30,0,-50353403,100,0,0,0",
"T3,20,10,-35111222,0,0,0,0",
"T3,20,-10,-35111222,0,0,0,0",
"T3,20,-10,35111222,0,0,0,0",
"T3,20,18000000,0,-3600000,0,0,0",
"T3,20,-18000000,0,3600000,0,0,0",
"T3,20,478000000,0,-9600000,0,0,0",
"T3,20,-478000000,0,9600000,0,0,0",
"T3,3338,225,-513,-38,0,0,0",
"T3,3338,-225,513,38,0,0,0",
"T3,2623,254,-563,173,0,0,0",
"T3,2623,-254,563,-173,0,0,0",
"T3,3000,100,-300,300,0,0,0",
"T3,3000,-100,300,-300,0,0,0",
"T3,3000,100,-300,300,0,0,0",
"T3,3000,-100,300,-300,0,0,0",
"T3,1000,500000,-1000000,0,0,0,0",
"T3,1000,-500000,1000000,0,0,0,0",
"T3,1000,500000,-1000000,5,0,0,0",
"T3,1000,-500000,1000000,-5,0,0,0",
]



print("connected")

response = query(the_port, 'V\r')
print(response)



last_command = ""

for command in command_list:
    block(ad)
    response = str(query(the_port, command + '\r'))
    print(last_command + " :: " + response.strip())
    last_command = command
    
print(last_command + " :: ")


ad.disconnect()             # Close serial port to AxiDraw

'''
Expected results log T3, 2023-05-07

T,22,S,5,C,45297792,R,490123456,P,5
T,22,S,5,C,2102185855,R,-490123456,P,-5
T,2,S,1,C,0,R,1073741824,P,1
T,2,S,1,C,2147483647,R,-1073741824,P,-1
T,100,S,31,C,94673512,R,1980066666,P,31
T,100,S,31,C,2052810135,R,-1980066666,P,-31
T,19512,S,6341,C,1855618940,R,2093842477,P,6341
T,19512,S,6341,C,291864707,R,-2093842477,P,-6341
T,19512,S,4761,C,1311430011,R,1603719021,P,1889
T,19512,S,4761,C,836053636,R,-1603719021,P,-1889
T,30000,S,7542,C,458869335,R,2099882501,P,3144
T,30000,S,7542,C,1688614312,R,-2099882501,P,-3144
T,35000,S,14171,C,1335592123,R,1872246545,P,-7037
T,35000,S,14171,C,811891524,R,-1872246545,P,7037
T,35000,S,14171,C,1235592124,R,1872246545,P,-7037
T,35000,S,14172,C,711891524,R,-1872246545,P,7038
T,20,S,5,C,138792703,R,490123456,P,5
T,20,S,5,C,2008690943,R,-490123456,P,-5
T,85,S,29,C,1142286978,R,1474952488,P,29
T,85,S,29,C,1005196669,R,-1474952488,P,-29
T,83,S,29,C,257219053,R,1440245682,P,29
T,83,S,28,C,978773657,R,1423065814,P,28
T,22,S,5,C,222764444,R,513240122,P,5
T,22,S,4,C,132168859,R,-467006790,P,-4
T,22,S,4,C,2015314788,R,467006790,P,4
T,22,S,5,C,1924719203,R,-513240122,P,-5
T,69,S,44,C,700483895,R,1428293187,P,44
T,69,S,44,C,1446999752,R,-1428293187,P,-44
T,30,S,10,C,1184194885,R,1485425389,P,10
T,30,S,10,C,963288762,R,-1485425389,P,-10
T,30,S,10,C,1184644865,R,1485468905,P,10
T,30,S,10,C,963738742,R,-1485381873,P,-10
T,20,S,3,C,1567690391,R,-684668819,P,-3
T,20,S,3,C,1567689991,R,-684668839,P,-3
T,20,S,3,C,579793256,R,684668819,P,3
T,20,S,3,C,2002450944,R,-666600000,P,-3
T,20,S,3,C,145032703,R,666600000,P,3
T,20,S,4,C,1054967296,R,-1347600000,P,-2
T,20,S,4,C,1092516351,R,1347600000,P,2
T,3338,S,111,C,2106467160,R,-213351133,P,-111
T,3338,S,111,C,41016487,R,213351133,P,111
T,2623,S,242,C,863356844,R,593428083,P,242
T,2623,S,242,C,1284126803,R,-593428083,P,-242
T,3000,S,628,C,30569056,R,1348650300,P,628
T,3000,S,628,C,2116914591,R,-1348650300,P,-628
T,3000,S,628,C,30569056,R,1348650300,P,628
T,3000,S,628,C,2116914591,R,-1348650300,P,-628
T,1000,S,232,C,863689983,R,-999000000,P,-232
T,1000,S,232,C,1283793664,R,999000000,P,232
T,1000,S,232,C,1697022483,R,-996502500,P,-232
T,1000,S,232,C,450461164,R,996502500,P,232
'''








