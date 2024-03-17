# Main EBB hardware in the loop test script
#
# Called without any parameters, this script will run all EBB hardware tests
# A single sub test can be run by naming that test as the first parameter
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

import sys

# Our global test result output file
import test_log

# Import the test function from each test file
from test_ISR_math import test_ISR_math_run
from test_global_step_counter import test_global_step_counter_run

# Start off assuming all will pass. Any that fail will flip this
all_tests_pass = True

# Generate the test run output directory and init the test log file
test_log.tl_init()

# Test
if sys.argv[1] == "" or sys.argv[1] == "test_ISR_math":
    test_log.tl_print("Run test: test_ISR_math")
    if test_ISR_math_run("..\\test input data\\test_inputs_simple.csv") == False:
        all_tests_pass = False

# Test
if sys.argv[1] == "" or sys.argv[1] == "test_global_step_counter":
    test_log.tl_print("Run test: test_global_step_counter")
    if test_global_step_counter_run() == False:
        all_tests_pass = False


# Done running all tests
if all_tests_pass == True:
    test_log.tl_print("test_run: All tests passed")
else:
    test_log.tl_print("test_run: Some or all tests failed")

test_log.tl_print("test_run: Complete")
test_log.tl_close()