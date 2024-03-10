# Test file to show each function working
from saleae import automation
import os
import os.path
from datetime import datetime
import sys
import csv
import time

from pyaxidraw import axidraw

from ebb_serial_utility import query
from ebb_serial_utility import EBB_version_less_than
from saleae_capture_one import capture_command
from analyze_digital_csv import analyze_digital_csv
from analyze_digital_csv import extract_debug_uart
from analyze_digital_csv import compare_debug_uart

def test_test_run(test_input_path : str):
    # Create new test artifact directory
    # Each test will create a directory within this directory with its artifacts

    artifact_dir = os.path.join(os.getcwd(), f'./sample output/output-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')
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
    if EBB_version_less_than(EBB_version.decode("utf-8"), "3.0.0"):
        print("EBB version required is 3.0.0 or above, but found :" + EBB_version)
        return

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
    with open(test_input_path, "r", newline='') as test_input_file:
        reader = csv.reader(test_input_file, quotechar='"', doublequote=True, skipinitialspace=True)
        for param in reader:
            if (len(param) >= 5):
                if (param[0] != '0'):
                    print(param[2] + ": ", end='')
                    test_dir = os.path.join(artifact_dir, param[2])
                    #capture_command(param[1], test_dir, float(param[3]), param[4])
                    capture_command(param[1], test_dir, float(param[3]), the_port)
                    # Now perform the type of analysis appropriate for this test
                    # In this case, extract the debug UART data

                    # and then check to see if it matches what we have from the test input file
                    #check_debug_serial(analyzer_export_filepath, expected_params)
                    
                    if compare_debug_uart(extract_debug_uart(test_dir), param[4]):
                        print("Pass")
                    else:
                        print("Fail")

    ad.disconnect()             # Close serial port to AxiDraw
    test_input_file.close()
    print("Complete")
    return


# For calling the main function from the command line

test_test_run(sys.argv[1])