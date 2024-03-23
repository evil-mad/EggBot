# A test for measuring the shortest move time EBB firmware
#
# Connect to EBB, configure it for Saleae output capture
# There are basically three code paths (for stepper moves) through the ISR
# in the EBB firmware. We pick a representative command for each of them.
# In this case SM, LM and L3. 
# For each command, we pick a move duration and a stepper step speed, and
# send enough of that set of command parameters to get to a steady state.
# Then we keep the step speed and command the same, and send a slightly
# shorter move, sending enough to get to a steady state. We go from maybe 
# about 15ms/move to 2ms/move, and save all of that as one capture.
# Then we choose a different stepper speed, and do it again. Because stepper
# speed changes how much CPU time is spent in the ISR, it also changes how
# fast we can parse/process move commands. So we will get different behavior
# with 25 KHz steps compared to 1 KHz steps for example. 
# For each capture, we need to look through the step pulses looking for
# gaps larger than say 3 step times. The first move duration (15 to 2ms) where
# these gaps occur is 'too fast', so one ms slower is the fastest move for
# that combination of command and step rate. So for each step rate, and each 
# of the three commands, we will have a 'fastest allowed without FIFO underrun'
# move duration. This can then get summarized in the EBB documentation.
#
# Note that because we only use the step1 pulses for this test, we can
# run it on any version of EBB firmware. Now, some versions don't have
# all commands (i.e. v3.0.0 has L3) so we have to query the EBB at the
# start of the run and turn on/off various commands based on which version
# we see.

import os
import os.path
import csv
import time

from pyaxidraw import axidraw

import test_log
from ebb_serial_utility import query
from ebb_serial_utility import EBB_version_less_than
from saleae_capture_one import capture_command
from analyze_digital_csv import analyze_digital_csv

def test_shortest_move_run():
    all_tests_pass = True

    output_filepath = test_log.artifact_dir + f'test_shortest_move/'

    # Connect to EBB

    ad = axidraw.AxiDraw() # Initialize class
    ad.interactive()

    if not ad.connect():                # Open serial port to AxiDraw;
        test_log.tl_print("test_shortest_move: failed to connect")
        return False

    the_port = ad.plot_status.port
    if the_port is None:
        test_log.tl_print("test_shortest_move: failed to connect")
        return False
    
    the_port.reset_input_buffer()
    test_log.tl_print("test_shortest_move: connected")
    EBB_version = query(the_port, 'V\r')
    # Put the version string of the currently attached EBB into the test log
    test_log.tl_print(EBB_version.decode("utf-8"), False)
    if EBB_version_less_than(EBB_version.decode("utf-8"), "3.0.0"):
        version_3_or_above = False
    else:
        version_3_or_above = True
        

    # Turn on the debug features we need in order to run our tests
    # Note that for this test, we turn all debug outputs off, to give as accurate a result
    # as possible. This means there is a lot less data in the captures, but we can still
    # get a good 'signal' by looking for overly long gaps in the step signals
    response = str(query(the_port, "CU,250,0" + '\r'))
    #print(last_command + " :: " + response.strip())
    response = str(query(the_port, "CU,251,0" + '\r'))
    #print(last_command + " :: " + response.strip())
    response = str(query(the_port, "CU,257,0" + '\r'))
    #print(last_command + " :: " + response.strip())

    if version_3_or_above:
        move_commands = ["SM", "LM", "L3"]
    else:
        move_commands = ["SM", "LM"]

    move_duration_max = 15      # in milliseconds
    stepper_speed_max = 25000   # in steps/second
    stepper_speed_min = 2000    # in steps/second. Since we go down to 1ms move durations, we need at least 2 steps per move
    command_repeats = 20

    for move_command in move_commands:
        speed_decrement = int(stepper_speed_max/2)

        stepper_speed = stepper_speed_max
        while stepper_speed >= stepper_speed_min:
            # For each run through the various move durations at a given stepper speed, there is a threshold
            # for the max low period of 2.5 * the expected time between steps pulses. The first command length which gives
            # a max low period of more than this threshold is the final result - i.e. the move duration which, at this
            # step rate, is no longer able to sustain smooth steps. So the previous move duration is the shortest move
            # duration for this step rate which can sustain smooth motion, and thus is the value that gets entered in
            # our final output table.
            maximum_allowed_time_between_steps_us = 2.5 * (1.0/(stepper_speed / 1000.0) * 1000)
            # This records the latest move duration which does NOT go above the threshold
            shortest_smooth_move_ms = move_duration_max + 1     # Start with an 'invalid' value (16) to alert if we never get a duration lower
            #print("Speed = " + str(stepper_speed) + " decrement = " + str(speed_decrement))
            for move_duration in range(move_duration_max, 0, -1):
                # build up the full command SM,1000,25000,250000
                #                           SM, 100,2500,2500
                if move_command == "SM":
                    sm_steps = int(stepper_speed*(move_duration/1000.0))
                    full_command = move_command + "," + str(move_duration) + "," + str(sm_steps) + "," + str(sm_steps)
                if move_command == "LM":
                    lm_rate = int(85899.35 * stepper_speed)
                    if lm_rate > 2147483647:
                        lm_rate = 2147483647
                    lm_steps = int(stepper_speed*(move_duration/1000.0))
                    lm_accel = 1            # Hard coding 1 as our accel, so that the math in the ISR has something to do, but don't actually change the rate much
                    full_command = move_command + "," + str(lm_rate) + "," + str(lm_steps) + "," + str(lm_accel) + "," + str(lm_rate) + "," + str(lm_steps) + "," + str(lm_accel)
                if move_command == "L3":
                    l3_rate = int(85899.35 * stepper_speed)
                    if l3_rate > 2147483647:
                        l3_rate = 2147483647
                    l3_steps = int(stepper_speed*(move_duration/1000.0))
                    l3_accel = 1            # Hard coding 1 as our accel, so that the math in the ISR has something to do, but don't actually change the rate much
                    l3_jerk = 1             # Hard coding 1 as our jerk, so that the math in the ISR has something to do, but don't actually change the rate much
                    full_command = move_command + "," + str(l3_rate) + "," + str(l3_steps) + "," + str(l3_accel) + "," + str(l3_jerk) + "," + str(l3_rate) + "," + str(l3_steps) + "," + str(l3_accel) + "," + str(l3_jerk)
                #print(full_command)

                test_log.tl_print("test_shortest_move: " + full_command + " (" + str(stepper_speed) + "): ", False)
                test_dir = os.path.join(output_filepath, move_command + "-" + str(stepper_speed) + "-" + str(move_duration))
                
                def ebb_command_function():
                    # Send all the commands
                    for i in range(command_repeats):
                        query(the_port, full_command + '\r')

                    # Give the commands some time to run and be captured
                    time.sleep(1.0)

                # For this length of command, figure out how long to capture for (in seconds)
                capture_length_S = command_repeats * (move_duration / 1000) + 1
                capture_command(ebb_command_function, test_dir, capture_length_S, the_port)

                # Now perform the type of analysis appropriate for this test
                # In this case what we want is the longest period where the step 1 signal was low
                # (other than before or after the moves)

                res_dict = analyze_digital_csv(test_dir + "/digital.csv")
                measured_max_low_time = (int(res_dict["Step1_MaxLowTimeUS"]))
                test_log.tl_print(str(measured_max_low_time))

                if measured_max_low_time < maximum_allowed_time_between_steps_us and measured_max_low_time > 0:
                    # Update the new shortest smooth move to the one we just analyzed
                    shortest_smooth_move_ms = move_duration
                
            stepper_speed = stepper_speed - speed_decrement
            speed_decrement = int(stepper_speed/2)
            test_log.tl_print("Shortest smooth move in ms was : " + str(shortest_smooth_move_ms))


    #if compare_debug_uart(extract_debug_uart(test_dir), param[4]):
    #    test_log.tl_print("Pass")
    #else:
    #    test_log.tl_print("Fail")
    #    all_tests_pass = False

    ad.disconnect()             # Close serial port to AxiDraw

    test_log.tl_print("test_shortest_move: Complete")
    return all_tests_pass
