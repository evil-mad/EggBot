# This program takes a number of command line parameters and captures a bunch of Saleae files.
# It is used as a component of the EBB test system.
#
# The EBB must be connected to the Saleae in the following manner:
# Saleae    EBB     Label
# D0        D6      Step1
# D1        D7      Dir1
# D2        D4      Step2
# D3        D5      Dir2
# D4        D1      In ISR
# D5        D0      Next Cmd
# D6        A1      FIFO Empty
# D7        C6      Dbg Serial
# D8        C0      Parsing
#
# This code requires version 3.0.0-a39 or above on the EBB
# This code requires that the Saleae Logic2 software have the Automation feature turned on and 
# software installed as called out here : https://saleae.github.io/logic2-automation/
# This code requires a 16 channel Saleae - either the "Logic 16" or the "Logic Pro 16" as we
# need more than 8 simultaneous input channels.
#
# Parameters:
# saleae_capture.py <input file> <
#
# <input file> is the name of an input file to read in 
#   Each line of the input file must contain
#   <EBB command to send> <



from saleae import automation
import os
import os.path
from datetime import datetime
import sys
import time
import csv

from pyaxidraw import axidraw
from plotink import ebb_motion
from plotink import ebb_serial



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

# Pass in a filepath and a string of the 5 parameters to check
# This function will open the .csv file with the debug serial data,
# pull out the five parameters, and check them. 
# Prints out either pass or fail
def check_debug_serial(serial_filepath, parameter_str):
    output_str = ''

    with open(serial_filepath, newline='') as f:
        reader = csv.reader(f)
        try:
            for row in reader:
                if reader.line_num > 1:
                    output_str = output_str + row[4]

        except csv.Error as e:
            sys.exit('file {}, line {}: {}'.format(serial_filepath, reader.line_num, e))

    # we now have a string like "T,000061A9,S,000004D2,C,06511930,R,06516DB0,P,000004D2"
    # Convert that into separate variables and decimal values

    #print(int.from_bytes(bytes(output_str[0:1],"utf-8"), "big", signed=False))
    #print(int.from_bytes(bytes(output_str[1:2],"utf-8"), "big", signed=False))
    #print(int.from_bytes(bytes(output_str[2:3],"utf-8"), "big", signed=False))
    #print(int.from_bytes(bytes(output_str[3:4],"utf-8"), "big", signed=False))


    #move_ticks = int.from_bytes(bytes(output_str[0:4],"utf-8"), "big", signed=False)
    #move_steps = int.from_bytes(bytes(output_str[4:8],"utf-8"), "big", signed=False)
    #move_accumulator1 = int.from_bytes(bytes(output_str[8:12],"utf-8"), "big", signed=False)
    #move_rate1 = int.from_bytes(bytes(output_str[12:16],"utf-8"), "big", signed=False)
    #move_position1 = int.from_bytes(bytes(output_str[16:20],"utf-8"), "big", signed=False)

    # We now have a string that looks like "0x000x000x610xA80x000x000x040xD20x060x500xC40xB00x060x510x6D0xB00x000x000x040xD20x0A"

    # I know this is super inefficient and inelegant. When you have time, rewrite to be cleaner. But this works.
    move_ticks = int(output_str[2:4], 16)
    move_ticks = (move_ticks * 256) + int(output_str[6:8], 16)
    move_ticks = (move_ticks * 256) + int(output_str[10:12], 16)
    move_ticks = (move_ticks * 256) + int(output_str[14:16], 16)
    move_steps = int(output_str[18:20], 16)
    move_steps = (move_steps * 256) + int(output_str[22:24], 16)
    move_steps = (move_steps * 256) + int(output_str[26:28], 16)
    move_steps = (move_steps * 256) + int(output_str[30:32], 16)
    move_accumulator1 = int(output_str[34:36], 16) 
    move_accumulator1 = (move_accumulator1 * 256) + int(output_str[38:40], 16)
    move_accumulator1 = (move_accumulator1 * 256) + int(output_str[42:44], 16)
    move_accumulator1 = (move_accumulator1 * 256) + int(output_str[46:48], 16)
    move_rate1 = int(output_str[50:52], 16)
    move_rate1 = (move_rate1 * 256) + int(output_str[54:56], 16)
    move_rate1 = (move_rate1 * 256) + int(output_str[58:60], 16)
    move_rate1 = (move_rate1 * 256) + int(output_str[62:64], 16)
    move_position1 = int(output_str[66:68], 16)
    move_position1 = (move_position1 * 256) + int(output_str[70:72], 16)
    move_position1 = (move_position1 * 256) + int(output_str[74:76], 16)
    move_position1 = (move_position1 * 256) + int(output_str[78:80], 16)

    #print("ticks=" + str(move_ticks))
    #print("steps=" + str(move_steps))
    #print("acc1=" + str(move_accumulator1))
    #print("rate1=" + str(move_rate1))
    #print("pos1=" + str(move_position1))

    # Parse apart the correct parameter string
    para_list = parameter_str.split(",")
    para_ticks = int(para_list[0])
    para_steps = int(para_list[1])
    para_accumulator1 = int(para_list[2])
    para_rate1 = int(para_list[3])
    para_position1 = int(para_list[4])

    if (move_ticks == para_ticks) & (move_steps == para_steps) & (move_accumulator1 == para_accumulator1) & (move_rate1 == para_rate1) & (move_position1 == para_position1):
        print ("Pass")

    if (move_ticks != para_ticks):
        print("Fail: measured ticks " + str(move_ticks) + " != expected ticks " + str(para_ticks))

    if (move_steps != para_steps):
        print("Fail: measured steps " + str(move_steps) + " != expected steps " + str(para_steps))
    
    if (move_accumulator1 != para_accumulator1):
        print("Fail: measured accumulator1 " + str(move_accumulator1) + " != expected accumulator1 " + str(para_accumulator1))
    
    if (move_rate1 != para_rate1):
        print("Fail: measured rate1 " + str(move_rate1) + " != expected rate1 " + str(para_rate1))
    
    if (move_position1 != para_position1):
        print("Fail: measured position1 " + str(move_position1) + " != expected position1 " + str(para_position1))




# Pass in an EBB command a file path name and a time and this function
# will start the capture (with length of the time), send the EBB command,
# and then save off the resulting Saleae capture file (.sal) as well as the
# .csv file of the bytes on the debug serial port into the capture_dir_name 
# directory

def capture_command(EBB_command, capture_dir_name, capture_time, expected_params):
    # Connect to the running Logic 2 Application on port `10430`.
    # Alternatively you can use automation.Manager.launch() to launch a new Logic 2 process - see
    # the API documentation for more details.
    # Using the `with` statement will automatically call manager.close() when exiting the scope. If you
    # want to use `automation.Manager` outside of a `with` block, you will need to call `manager.close()` manually.
    with automation.Manager.connect(port=10430) as manager:

        # Configure the capturing device to record on digital channels 0, 1, 2, and 3,
        # with a sampling rate of 10 MSa/s, and a logic level of 3.3V.
        # The settings chosen here will depend on your device's capabilities and what
        # you can configure in the Logic 2 UI.
        device_configuration = automation.LogicDeviceConfiguration(
            enabled_digital_channels=[0, 1, 2, 3, 4, 5, 6, 7, 8],
            digital_sample_rate=32_000_000,
        )

        # Record 5 seconds of data before stopping the capture
        capture_configuration = automation.CaptureConfiguration(
            capture_mode=automation.TimedCaptureMode(duration_seconds=capture_time)
        )

        # Start a capture - the capture will be automatically closed when leaving the `with` block
        # Note: The serial number 'F4241' is for the Logic Pro 16 demo device.
        #       To use a real device, you can:
        #         1. Omit the `device_id` argument. Logic 2 will choose the first real (non-simulated) device.
        #         2. Use the serial number for your device. See the "Finding the Serial Number
        #            of a Device" section for information on finding your device's serial number.
        with manager.start_capture(
                device_configuration=device_configuration,
                capture_configuration=capture_configuration) as capture:

            # The capture has started. We now need to send our EBB command:
            response = str(query(the_port, EBB_command + '\r'))
            print(last_command + " :: " + response.strip())

            # Wait until the capture has finished
            capture.wait()

            analyzers = []

            # Add an analyzer to the capture
            async_analyzer = capture.add_analyzer('Async Serial', label=f'ISR Serial', settings={
                'Input Channel': 7,
                'Bit Rate (Bits/s)': 4000000,
            })
            analyzers.append(automation.DataTableExportConfiguration(async_analyzer, automation.RadixType.HEXADECIMAL))

            # Create a directory name to store our output files in
            # output_dir = os.path.join(os.getcwd(), f'output-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')
            output_dir = os.path.join(os.getcwd(), capture_dir_name)
            os.makedirs(output_dir, exist_ok = True)

            # Export analyzer data to a CSV file
            analyzer_export_filepath = os.path.join(output_dir, 'debug_serial.csv')
            capture.export_data_table(
                filepath=analyzer_export_filepath,
                analyzers=analyzers
            )

            # Export raw digital data to a CSV file
            capture.export_raw_data_csv(directory=output_dir, digital_channels=[0, 1, 2, 3, 4, 5, 6, 7, 8])

            # Save the capture to a file
            capture_filepath = os.path.join(output_dir, 'capture.sal')
            capture.save_capture(filepath=capture_filepath)

            # Gather up the debug serial output
            check_debug_serial(analyzer_export_filepath, expected_params)

            # Save off text file with EBB command and parameters
            with open(os.path.join(output_dir, 'param.csv'), 'w') as param_file:
                param_file.write(EBB_version.decode() + '\n')
                param_file.write(EBB_command + '\n')
                param_file.write(output_dir + '\n')
                param_file.write(capture_dir_name + '\n')
                param_file.write(str(capture_time) + '\n')


# Temporary list of commands to send (will get from file eventually)
param_list = [
["SM,100,100,0,3",     "test1", 1, "2500,100,20054816,85899344,100"],
["SM,100,-100,0,3",    "test2", 1, "2500,100,20035616,85899344,4294967196"],
["SM,100,0,100,3",     "test3", 1, "-1,-1,-1,-1,-1"],
#["SM,100,0,-100,3",    "test4" ,1, ""],
#["SM,100,100,100,3",   "test5" ,1, ""],
#["SM,100,-100,-100,3", "test6" ,1, ""],
#["XM,100,100,0,3",     "test1" ,1, ""],
#["XM,100,-100,0,3",    "test1" ,1, ""],
#["XM,100,0,100,3",     "test1" ,1, ""],
#["XM,100,0,-100,3",    "test1" ,1, ""],
#["XM,100,100,100,3",   "test1" ,1, ""],
#["XM,100,-100,-100,3", "test1" ,1, ""],
]


#
# Connect to EBB
#

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

print("connected")

EBB_version = query(the_port, 'V\r')

last_command = ""

response = str(query(the_port, "CU,250,1" + '\r'))
print(last_command + " :: " + response.strip())
response = str(query(the_port, "CU,251,1" + '\r'))
print(last_command + " :: " + response.strip())
response = str(query(the_port, "CU,257,1" + '\r'))
print(last_command + " :: " + response.strip())


for param in param_list:
    print(param[1], end='')
    capture_command(param[0], param[1], param[2], param[3])


#    #block(ad)
#    response = str(query(the_port, command + '\r'))
#    print(last_command + " :: " + response.strip())
#    last_command = command
    
#print(last_command + " :: ")

print("Complete")

ad.disconnect()             # Close serial port to AxiDraw



