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
# This code requires version 3.0.0-a40 or above on the EBB
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

# TODO List
#   Check for 0x0A at end of debug uart string to check for framing problems
#   Don't require Saleae Logic to be running but fire it up here 

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
                    if row[4][0:2] != '0x':
                        print('Debug UART csv parse error: csv line ' + str(reader.line_num) + ' is not 0x but instead is ' + row[4][0:2])
                    else:
                        # Put all four bytes of each 32 bit int together, with a comma between the 32 bit values
                        if (reader.line_num >= 6) & ((reader.line_num - 6) % 4 == 0):
                            output_str = output_str + ',' + row[4][2:4]
                        else:
                            output_str = output_str + row[4][2:4]

        except csv.Error as e:
            sys.exit('file {}, line {}: {}'.format(serial_filepath, reader.line_num, e))

    # We now have a string that looks like "000009C5,00000064,051EA590,051EB850,00000064,00000000,00000000,00000000,00000000,0A"
    dbg_list = output_str.split(',')

    if len(dbg_list) < 10:
        print('Error: not enough fields in debug UART output:' + output_str)
        print(dbg_list)
        print(str(len(dbg_list)))
        move_ticks = 0
        move_steps1 = 0
        move_accumulator1 = 0
        move_rate1 = 0
        move_position1 = 0
        move_steps2 = 0
        move_accumulator2 = 0
        move_rate2 = 0
        move_position2 = 0
    else:
        move_ticks =        int(dbg_list[0], 16)
        move_steps1 =       int(dbg_list[1], 16)
        move_accumulator1 = int(dbg_list[2], 16) 
        move_rate1 =        int(dbg_list[3], 16)
        move_position1 =    int(dbg_list[4], 16)
        move_steps2 =       int(dbg_list[5], 16)
        move_accumulator2 = int(dbg_list[6], 16) 
        move_rate2 =        int(dbg_list[7], 16)
        move_position2 =    int(dbg_list[8], 16)
        if dbg_list[9] != '0A':
            print('Error: incorrect final field in debug UART output:' + dbg_list[9])

        # Rate and Position are signed numbers, so take care of negating if their top most bit is set
        if (move_rate1 > 0x7FFFFFFF):
            move_rate1 = move_rate1 - 0x100000000
        if (move_position1 > 0x7FFFFFFF):
            move_position1 = move_position1 - 0x100000000
        if (move_rate2 > 0x7FFFFFFF):
            move_rate2 = move_rate2 - 0x100000000
        if (move_position2 > 0x7FFFFFFF):
            move_position2 = move_position2 - 0x100000000

    #print("ticks=" + str(move_ticks))
    #print("steps=" + str(move_steps))
    #print("acc1=" + str(move_accumulator1))
    #print("rate1=" + str(move_rate1))
    #print("pos1=" + str(move_position1))

    para_ticks = 0
    para_steps1 = 0
    para_accumulator1 = 0
    para_rate1 = 0
    para_position1 = 0
    para_steps2 = 0
    para_accumulator2 = 0
    para_rate2 = 0
    para_position2 = 0

    # Parse apart the correct parameter string
    para_list = parameter_str.split(",")
    if len(para_list) >= 9:
        if (para_list[0] != "") & (para_list[0] != '*'):
            para_ticks =         int(para_list[0])
        if (para_list[1] != "") & (para_list[1] != '*'):
            para_steps1 =        int(para_list[1])
        if (para_list[2] != "") & (para_list[2] != '*'):
            para_accumulator1 =  int(para_list[2])
        if (para_list[3] != "") & (para_list[3] != '*'):
            para_rate1 =         int(para_list[3])
        if (para_list[4] != "") & (para_list[4] != '*'):
            para_position1 =     int(para_list[4])
        if (para_list[5] != "") & (para_list[5] != '*'):
            para_steps2 =        int(para_list[5])
        if (para_list[6] != "") & (para_list[6] != '*'):
            para_accumulator2 =  int(para_list[6])
        if (para_list[7] != "") & (para_list[7] != '*'):
            para_rate2 =         int(para_list[7])
        if (para_list[8] != "") & (para_list[8] != '*'):
            para_position2 =     int(para_list[8])

        if ((move_ticks == para_ticks) | (move_ticks - 1 == para_ticks) | (para_list[0] == '*'))                \
        &                                                                      \
        ((move_steps1 == para_steps1) | (para_list[1] == '*'))                 \
        &                                                                      \
        ((move_accumulator1 == para_accumulator1) | (para_list[2] == '*'))     \
        &                                                                      \
        ((move_rate1 == para_rate1) | (para_list[3] == '*'))                   \
        &                                                                      \
        ((move_position2 == para_position2) | (para_list[4] == '*'))           \
        &                                                                      \
        ((move_steps2 == para_steps2) | (para_list[5] == '*'))                 \
        &                                                                      \
        ((move_accumulator2 == para_accumulator2) | (para_list[6] == '*'))     \
        &                                                                      \
        ((move_rate2 == para_rate2) | (para_list[7] == '*'))                   \
        &                                                                      \
        ((move_position2 == para_position2) | (para_list[8] == '*')):
                print ("Pass")
        else:
            if ((move_ticks != para_ticks) & (para_list[0] != '*')):
                print("Fail: measured ticks " + str(move_ticks) + " != expected ticks " + str(para_ticks))

            if ((move_steps1 != para_steps1) & (para_list[1] != '*')):
                print("Fail: measured steps1 " + str(move_steps1) + " != expected steps1 " + str(para_steps1))
            
            if ((move_accumulator1 != para_accumulator1) & (para_list[2] != '*')):
                print("Fail: measured accumulator1 " + str(move_accumulator1) + " != expected accumulator1 " + str(para_accumulator1))
            
            if ((move_rate1 != para_rate1) & (para_list[3] != '*')):
                print("Fail: measured rate1 " + str(move_rate1) + " != expected rate1 " + str(para_rate1))
            
            if ((move_position1 != para_position1) & (para_list[4] != '*')):
                print("Fail: measured position1 " + str(move_position1) + " != expected position1 " + str(para_position1))

            if ((move_steps2 != para_steps2) & (para_list[5] != '*')):
                print("Fail: measured steps2 " + str(move_steps2) + " != expected steps2 " + str(para_steps2))
            
            if ((move_accumulator2 != para_accumulator2) & (para_list[6] != '*')):
                print("Fail: measured accumulator2 " + str(move_accumulator2) + " != expected accumulator2 " + str(para_accumulator2))
            
            if ((move_rate2 != para_rate2) & (para_list[7] != '*')):
                print("Fail: measured rate2 " + str(move_rate2) + " != expected rate2 " + str(para_rate2))
            
            if ((move_position2 != para_position2) & (para_list[8] != '*')):
                print("Fail: measured position2 " + str(move_position2) + " != expected position2 " + str(para_position2))
    else:
        print("Incomplete test description. Measured values = ", end='')
        print(str(move_ticks) + "," + str(move_steps1) + "," + str(move_accumulator1) + "," + str(move_rate1) + "," + str(move_position1), end='')
        print("," + str(move_steps2) + "," + str(move_accumulator2) + "," + str(move_rate2) + "," + str(move_position2))


# Pass in an EBB command a file path name and a time and this function
# will start the capture (with length of the time), send the EBB command,
# and then save off the resulting Saleae capture file (.sal) as well as the
# .csv file of the bytes on the debug serial port into the capture_dir_name 
# directory

def capture_command(EBB_command : str, capture_dir_path : str, capture_time : float, expected_params : str):
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
            # But wait a bit first in case there is a delay starting up the capture
            time.sleep(0.2)
            response = str(query(the_port, EBB_command + '\r'))
            # TODO: Confirm that response was "OK\r\n", print error and actual response if not
            #print(last_command + " :: " + response.strip())

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
            os.makedirs(capture_dir_path, exist_ok = True)

            # Export analyzer data to a CSV file
            analyzer_export_filepath = os.path.join(capture_dir_path, 'debug_serial.csv')
            capture.export_data_table(
                filepath=analyzer_export_filepath,
                analyzers=analyzers
            )

            # Export raw digital data to a CSV file 
            capture.export_raw_data_csv(directory=capture_dir_path, digital_channels=[0, 1, 2, 3, 4, 5, 6, 7, 8])

            # Save the capture to a file
            capture_filepath = os.path.join(capture_dir_path, 'capture.sal')
            capture.save_capture(filepath=capture_filepath)

            # Gather up the debug serial output
            check_debug_serial(analyzer_export_filepath, expected_params)

            # Save off text file with EBB command and parameters
            with open(os.path.join(capture_dir_path, 'param.csv'), 'w') as param_file:
                param_file.write(EBB_version.decode() + '\n')
                param_file.write(EBB_command + '\n')
                param_file.write(capture_dir_path + '\n')
                param_file.write(str(capture_time) + '\n')



# Create new test artifact directory
# Each test will create a directory within this directory with its artifacts

artifact_dir = os.path.join(os.getcwd(), f'output-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')
os.makedirs(artifact_dir, exist_ok = True)

# Connect to EBB

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

# Turn on the debug features we need in order to run our tests

response = str(query(the_port, "CU,250,1" + '\r'))
print(last_command + " :: " + response.strip())
response = str(query(the_port, "CU,251,1" + '\r'))
print(last_command + " :: " + response.strip())
response = str(query(the_port, "CU,257,1" + '\r'))
print(last_command + " :: " + response.strip())

# Walk through each line of the tests from the test input file
# and execute them

#for data_line in test_input_file:
with open(sys.argv[1], "r", newline='') as test_input_file:
    reader = csv.reader(test_input_file, quotechar='"', doublequote=True, skipinitialspace=True)
    for param in reader:
        if (len(param) >= 5):
            if (param[0] != '0'):
                print(param[2] + ": ", end='')
                test_dir = os.path.join(artifact_dir, param[2])
                capture_command(param[1], test_dir, float(param[3]), param[4])

ad.disconnect()             # Close serial port to AxiDraw
test_input_file.close()
print("Complete")
