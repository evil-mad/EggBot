# A test for ISR math in v3.0.0 and above EBB firmware
#
# The caller passes in a CSV test file, which contains all of the commands to send
# as well as the expected results from the testing.
# We configure the EBB, start a Saleae capture, then send a command. When that
# command is done, we save the capture, extract the debug ISR UART variable values,
# and then compare that to the expected results. If they match that test passes.
# This is repeated for each line in the input test file.

import os
import os.path
import csv

from pyaxidraw import axidraw

import test_log
from ebb_serial_utility import query
from ebb_serial_utility import EBB_version_less_than
from saleae_capture_one import capture_command
from analyze_digital_csv import extract_debug_uart
from analyze_digital_csv import compare_debug_uart

def test_ISR_math_run(test_input_path : str):
    all_tests_pass = True

    input_filepath = os.path.join(os.getcwd(), test_input_path)
    output_filepath = test_log.artifact_dir + f'test_ISR_math/'

    # Connect to EBB

    ad = axidraw.AxiDraw() # Initialize class
    ad.interactive()

    if not ad.connect():                # Open serial port to AxiDraw;
        test_log.tl_print("test_ISR_math: failed to connect")
        return False

    the_port = ad.plot_status.port
    if the_port is None:
        test_log.tl_print("test_ISR_math: failed to connect")
        return False

    the_port.reset_input_buffer()
    test_log.tl_print("test_ISR_math: connected")
    EBB_version = query(the_port, 'V\r')
    if EBB_version_less_than(EBB_version.decode("utf-8"), "3.0.0"):
        test_log.tl_print("test_ISR_math: EBB version required is 3.0.0 or above, but found :" + EBB_version)
        return False

    # Turn on the debug features we need in order to run our tests

    response = str(query(the_port, "CU,250,1" + '\r'))
    #print(last_command + " :: " + response.strip())
    response = str(query(the_port, "CU,251,1" + '\r'))
    #print(last_command + " :: " + response.strip())
    response = str(query(the_port, "CU,257,1" + '\r'))
    #print(last_command + " :: " + response.strip())

    # Walk through each line of the tests from the test input file
    # and execute them

    #for data_line in test_input_file:
    with open(input_filepath, "r", newline='') as test_input_file:
        reader = csv.reader(test_input_file, quotechar='"', doublequote=True, skipinitialspace=True)
        for param in reader:
            if (len(param) >= 5):
                if (param[0] != '0'):
                    test_log.tl_print("test_ISR_math: " + param[2] + ": ", False)
                    test_dir = os.path.join(output_filepath, param[2])

                    def ebb_command_function():
                        # Send all the command
                        query(the_port, param[1] + '\r')

                    capture_command(ebb_command_function, test_dir, float(param[3]), the_port)
                    # Now perform the type of analysis appropriate for this test
                    # In this case, extract the debug UART data

                    # and then check to see if it matches what we have from the test input file
                    #check_debug_serial(analyzer_export_filepath, expected_params)
                    
                    if compare_debug_uart(extract_debug_uart(test_dir), param[4]):
                        test_log.tl_print("Pass")
                    else:
                        test_log.tl_print("Fail")
                        all_tests_pass = False

    ad.disconnect()             # Close serial port to AxiDraw
    test_input_file.close()
    test_log.tl_print("test_ISR_math: Complete")
    return all_tests_pass
