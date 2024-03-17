# A test for EBB firmware
# This test clears the global position values, runs a series of commands,
# the checks the global position values and confirms that they are correct.

import time

from pyaxidraw import axidraw

import test_log
from ebb_serial_utility import query
from ebb_serial_utility import EBB_version_less_than

def test_global_step_counter_run():
    all_tests_pass = True

    # Connect to EBB

    ad = axidraw.AxiDraw() # Initialize class
    ad.interactive()

    if not ad.connect():                # Open serial port to AxiDraw;
        test_log.tl_print("test_global_step_counter: failed to connect")
        return False

    the_port = ad.plot_status.port
    if the_port is None:
        test_log.tl_print("test_global_step_counter: failed to connect")
        return False

    the_port.reset_input_buffer()
    test_log.tl_print("test_global_step_counter: Connected")
    EBB_version = query(the_port, 'V\r')
    if EBB_version_less_than(EBB_version.decode("utf-8"), "3.0.0"):
        test_log.tl_print("test_global_step_counter: EBB version required is 3.0.0 or above, but found :" + EBB_version)
        return False

    # Test LM command- specifically looking for step position errors 
    # Clear the step position at the beginning, and then check it after
    # each test.
    #
    # First just straight non-accelerating moves in a square
    test_log.tl_print('test_global_step_counter: Test1: ', False)

    query(the_port, "CS" + '\r')
    for x in range(10):
        query(the_port, "LM,85899350,100,0,85899350,100,0" + '\r')
    for x in range(10):
        query(the_port, "LM,85899350,-100,0,85899350,100,0" + '\r')
    for x in range(10):
        query(the_port, "LM,85899350,-100,0,85899350,-100,0" + '\r')
    for x in range(10):
        query(the_port, "LM,85899350,100,0,85899350,-100,0" + '\r')

    time.sleep(1.0)

    response = query(the_port, 'QS\r', True)
    if response != b'0,0\n':
        all_tests_pass = False
        test_log.tl_print('Fail, ' + response.decode("utf-8"))
    else:
        test_log.tl_print('Pass')


    # Now each set of moves is an accel down to zero and then back up again as separate commands
    test_log.tl_print('test_global_step_counter: Test2: ', False)

    for x in range(10):
        query(the_port, "LM,85899350,100,-34360,85899350,100,-34360" + '\r')
        query(the_port, "LM,0,100,34360,0,100,34360" + '\r')
    for x in range(10):
        query(the_port, "LM,85899350,-100,-34360,85899350,100,-34360" + '\r')
        query(the_port, "LM,0,-100,34360,0,100,34360" + '\r')
    for x in range(10):
        query(the_port, "LM,85899350,-100,-34360,85899350,-100,-34360" + '\r')
        query(the_port, "LM,0,-100,34360,0,-100,34360" + '\r')
    for x in range(10):
        query(the_port, "LM,85899350,100,-34360,85899350,-100,-34360" + '\r')
        query(the_port, "LM,0,100,34360,0,-100,34360" + '\r')

    time.sleep(1.0)
    response = query(the_port, 'QS\r', True)
    if response != b'0,0\n':
        all_tests_pass = False
        test_log.tl_print('Fail, ' + response.decode("utf-8"))
    else:
        test_log.tl_print('Pass')

    ad.disconnect()             # Close serial port to AxiDraw
    test_log.tl_print("test_global_step_counter: Complete")
    return all_tests_pass
