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