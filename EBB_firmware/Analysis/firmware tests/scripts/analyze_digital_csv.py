# Utility functions for analyzing Saleae logic captures for various things

import csv, sys

def analyze_digital_csv(file_pathname : str):

    line_time = 0.0
    step1 = 0
    dir1 = 0
    step2 = 0
    dir2 = 0
    last_line_time = 0.0
    last_step1 = 0
    last_dir1 = 0
    last_step2 = 0
    last_dir2 = 0

    step1_count = 0
    step2_count = 0

    step1_rising_time = 0.0
    step1_falling_time = 0.0
    step1_max_high_time = 0.0
    step1_min_high_time = 999.0
    step1_ave_high_time_acc = 0.0
    step1_ave_high_time = 0.0
    step1_max_low_time = 0.0
    step1_min_low_time = 999.0
    step1_ave_low_time_acc = 0.0
    step1_ave_low_time = 0.0
    step1_freq_acc = 0.0
    step1_ave_freq = 0.0

    step2_rising_time = 0.0
    step2_falling_time = 0.0
    step2_max_high_time = 0.0
    step2_min_high_time = 999.0
    step2_ave_high_time_acc = 0.0
    step2_ave_high_time = 0.0
    step2_max_low_time = 0.0
    step2_min_low_time = 999.0
    step2_ave_low_time_acc = 0.0
    step2_ave_low_time = 0.0
    step2_freq_acc = 0.0
    step2_ave_freq = 0.0


    with open(file_pathname, newline='') as f:
        reader = csv.reader(f)
        try:
            for row in reader:
                if reader.line_num > 1:
                    line_time = float(row[0])
                    step1 = row[1]
                    dir1 = row[2]
                    step2 = row[3]
                    dir2 = row[4]
                    in_isr = row[5]
                    cmd_load = row[6]
                    fifo_empty = row[7]
                    isr_serial = row[8]

                    if last_step1 == '0' and step1 == '1':
                        # count this step
                        step1_count = step1_count + 1

                        # record this rising edge time
                        step1_rising_time = line_time

                        # Only save statistics once we have both a rising and falling edge
                        if step1_rising_time > 0 and step1_falling_time > 0:
                            step1_low_time = step1_rising_time - step1_falling_time
                            step1_ave_low_time_acc = step1_ave_low_time_acc + step1_low_time
                            if step1_low_time < step1_min_low_time:
                                step1_min_low_time = step1_low_time
                            if step1_low_time > step1_max_low_time:
                                step1_max_low_time = step1_low_time

                            step1_freq_acc = step1_freq_acc + step1_low_time


                    if last_step1 == '1' and step1 == '0':
                        # record this falling edge time
                        step1_falling_time = line_time

                        # Only save statistics once we have both a rising and falling edge
                        if step1_rising_time > 0 and step1_falling_time > 0:
                            step1_high_time = step1_falling_time - step1_rising_time
                            step1_ave_high_time_acc = step1_ave_high_time_acc + step1_high_time
                            if step1_high_time < step1_min_high_time:
                                step1_min_high_time = step1_high_time
                            if step1_high_time > step1_max_high_time:
                                step1_max_high_time = step1_high_time

                            step1_freq_acc = step1_freq_acc + step1_high_time

                    if last_step2 == '0' and step2 == '1':
                        step2_count = step2_count + 1

                        # record this rising edge time
                        step2_rising_time = line_time

                        # Only save statistics once we have both a rising and falling edge
                        if step2_rising_time > 0 and step2_falling_time > 0:
                            step2_low_time = step2_rising_time - step2_falling_time
                            step2_ave_low_time_acc = step2_ave_low_time_acc + step2_low_time
                            if step2_low_time < step2_min_low_time:
                                step2_min_low_time = step2_low_time
                            if step2_low_time > step2_max_low_time:
                                step2_max_low_time = step2_low_time

                            step2_freq_acc = step2_freq_acc + step2_low_time

                    if last_step2 == '1' and step2 == '0':
                        # record this falling edge time
                        step2_falling_time = line_time

                        # Only save statistics once we have both a rising and falling edge
                        if step2_rising_time > 0 and step2_falling_time > 0:
                            step2_high_time = step2_falling_time - step2_rising_time
                            step2_ave_high_time_acc = step2_ave_high_time_acc + step2_high_time
                            if step2_high_time < step2_min_high_time:
                                step2_min_high_time = step2_high_time
                            if step2_high_time > step2_max_high_time:
                                step2_max_high_time = step2_high_time

                            step2_freq_acc = step2_freq_acc + step2_high_time


                    last_step1 = step1
                    last_dir1 = dir1
                    last_step2 = step2
                    last_dir2 = dir2    


                    #print(row)
                    #if reader.line_num >= 10:
                    #    sys.exit('done')

        except csv.Error as e:
            sys.exit('file {}, line {}: {}'.format(filename, reader.line_num, e))

    # Output all statistics we just collected as a dict to the caller

    return_dict = {
        "Step1_Count": step1_count,
        "Step1_MaxHighTimeUS" : step1_max_high_time*1000000,
        "Step1_MinHighTimeUS" : step1_min_high_time*1000000,
        "Step1_AveHigTimeUS" : (step1_ave_high_time_acc/step1_count)*1000000,
        "Step1_MaxLowTimeUS" : step1_max_low_time*1000000,
        "Step1_MinLowTimeUS" : step1_min_low_time*1000000,
        "Step1_AveLowTimeUS" : (step1_ave_low_time_acc/step1_count)*1000000,
        "Step1_AveFreqHZ" : 1.0/(step1_freq_acc/step1_count),
        "Step2_Count": step2_count,
        "Step2_MaxHighTimeUS" : step2_max_high_time*1000000,
        "Step2_MinHighTimeUS" : step2_min_high_time*1000000,
        "Step2_AveHigTimeUS" : (step2_ave_high_time_acc/step1_count)*1000000,
        "Step2_MaxLowTimeUS" : step2_max_low_time*1000000,
        "Step2_MinLowTimeUS" : step2_min_low_time*1000000,
        "Step2_AveLowTimeUS" : (step2_ave_low_time_acc/step1_count)*1000000,
        "Step2_AveFreqHZ" : 1.0/(step2_freq_acc/step1_count),
    }

    return return_dict

# This function extracts the debug uart ISR parameters from a saved analyzer .csv file.
# This function will open the .csv file with the debug serial data,
# pull out the five parameters, and return them in a dictionary
def extract_debug_uart(serial_filepath):
    output_str = ''

    with open(serial_filepath+'/debug_serial.csv', newline='') as f:
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

    return_dict = {
        "ticks": move_ticks,
        "steps1" : move_steps1,
        "accumulator1" : move_accumulator1,
        "rate1" : move_rate1,
        "position1" : move_position1,
        "steps2" : move_steps2,
        "accumulator2" : move_accumulator2,
        "rate2" : move_rate2,
        "position2" : move_position2
    }

    return return_dict

# See if the debug UART ISR values extracted from a capture match the expected
# values.
# extracted_dict is a dictionary output from extract_debug_uart()
# expected_str is a string representing the expected values, and is of the form
#            "2500,0,0,0,0,100,85894544,85899344,100"
# where the fields are: ticks, steps1, acc1, rate1, pos1, step2, acc2, rate2, pos2
# Returns True if they match or False if they don't
def compare_debug_uart(extracted_dict : dict, expected_str : str):
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
    para_list = expected_str.split(",")
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

        if ((extracted_dict["ticks"] == para_ticks) | (extracted_dict["ticks"] - 1 == para_ticks) | (para_list[0] == '*'))                \
        &                                                                      \
        ((extracted_dict["steps1"] == para_steps1) | (para_list[1] == '*'))                 \
        &                                                                      \
        ((extracted_dict["accumulator1"] == para_accumulator1) | (para_list[2] == '*'))     \
        &                                                                      \
        ((extracted_dict["rate1"] == para_rate1) | (para_list[3] == '*'))                   \
        &                                                                      \
        ((extracted_dict["position1"] == para_position1) | (para_list[4] == '*'))           \
        &                                                                      \
        ((extracted_dict["steps2"] == para_steps2) | (para_list[5] == '*'))                 \
        &                                                                      \
        ((extracted_dict["accumulator2"] == para_accumulator2) | (para_list[6] == '*'))     \
        &                                                                      \
        ((extracted_dict["rate2"] == para_rate2) | (para_list[7] == '*'))                   \
        &                                                                      \
        ((extracted_dict["position2"] == para_position2) | (para_list[8] == '*')):
                return True
        else:
            if ((extracted_dict["ticks"] != para_ticks) & (extracted_dict["ticks"] - 1 != para_ticks) & (para_list[0] != '*')):
                print("Fail: measured ticks " + str(extracted_dict["ticks"]) + " != expected ticks " + str(para_ticks))

            if ((extracted_dict["steps1"] != para_steps1) & (para_list[1] != '*')):
                print("Fail: measured steps1 " + str(extracted_dict["steps1"]) + " != expected steps1 " + str(para_steps1))
            
            if ((extracted_dict["accumulator1"] != para_accumulator1) & (para_list[2] != '*')):
                print("Fail: measured accumulator1 " + str(extracted_dict["accumulator1"]) + " != expected accumulator1 " + str(para_accumulator1))
            
            if ((extracted_dict["rate1"] != para_rate1) & (para_list[3] != '*')):
                print("Fail: measured rate1 " + str(extracted_dict["rate1"]) + " != expected rate1 " + str(para_rate1))
            
            if ((extracted_dict["position1"] != para_position1) & (para_list[4] != '*')):
                print("Fail: measured position1 " + str(extracted_dict["position1"]) + " != expected position1 " + str(para_position1))

            if ((extracted_dict["steps2"] != para_steps2) & (para_list[5] != '*')):
                print("Fail: measured steps2 " + str(extracted_dict["steps2"]) + " != expected steps2 " + str(para_steps2))
            
            if ((extracted_dict["accumulator2"] != para_accumulator2) & (para_list[6] != '*')):
                print("Fail: measured accumulator2 " + str(extracted_dict["accumulator2"]) + " != expected accumulator2 " + str(para_accumulator2))
            
            if ((extracted_dict["rate2"] != para_rate2) & (para_list[7] != '*')):
                print("Fail: measured rate2 " + str(extracted_dict["rate2"]) + " != expected rate2 " + str(para_rate2))
            
            if ((extracted_dict["position2"] != para_position2) & (para_list[8] != '*')):
                print("Fail: measured position2 " + str(extracted_dict["position2"]) + " != expected position2 " + str(para_position2))
            
            return False
    else:
        print("Incomplete test description. Measured values = ", end='')
        print(str(extracted_dict["ticks"]) + "," + str(extracted_dict["steps1"]) + "," + str(extracted_dict["accumulator1"]) + "," + str(extracted_dict["rate1"]) + "," + str(extracted_dict["position1"]), end='')
        print("," + str(extracted_dict["steps2"]) + "," + str(extracted_dict["accumulator2"]) + "," + str(extracted_dict["rate2"]) + "," + str(extracted_dict["position2"]))
        
        return False
