import csv, sys

filename = 'output-2024-01-07_15-15-08'

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


with open(filename+'/digital.csv', newline='') as f:
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

# print out all the statistics we just computed

print("Step 1:")
print("  Count: " + str(step1_count) + " steps")
print("  Max High Time: " + str(int(step1_max_high_time*1000000)) + " uS")
print("  Min High Time: " + str(int(step1_min_high_time*1000000)) + " uS")
print("  Ave High Time: " + str(int((step1_ave_high_time_acc/step1_count)*1000000)) + " uS")
print("  Max Low Time: " + str(int(step1_max_low_time*1000000)) + " uS")
print("  Min Low Time: " + str(int(step1_min_low_time*1000000)) + " uS")
print("  Ave Low Time: " + str(int((step1_ave_low_time_acc/step1_count)*1000000)) + " uS")
print("  Ave Frequency: " + str(1.0/(step1_freq_acc/step1_count)) + " Hz")

print("Step 2:")
print("  Count: " + str(step2_count) + " steps")
print("  Max High Time: " + str(int(step2_max_high_time*1000000)) + " uS")
print("  Min High Time: " + str(int(step2_min_high_time*1000000)) + " uS")
print("  Ave High Time: " + str(int((step2_ave_high_time_acc/step2_count)*1000000)) + " uS")
print("  Max Low Time: " + str(int(step2_max_low_time*1000000)) + " uS")
print("  Min Low Time: " + str(int(step2_min_low_time*1000000)) + " uS")
print("  Ave Low Time: " + str(int((step2_ave_low_time_acc/step2_count)*1000000)) + " uS")
print("  Ave Frequency: " + str(1.0/(step2_freq_acc/step2_count)) + " Hz")

'''
output_str = ''

with open(filename+'/async_serial_export.csv', newline='') as f:
    reader = csv.reader(f)
    try:
        for row in reader:
            if reader.line_num > 1:
                output_str = output_str + row[4]

    except csv.Error as e:
        sys.exit('file {}, line {}: {}'.format(filename, reader.line_num, e))

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

move_ticks = int(output_str[2:10], 16)
move_steps = int(output_str[13:21], 16)
move_accumulator1 = int(output_str[24:32], 16) 
move_rate1 = int(output_str[35:43], 16)
move_position1 = int(output_str[46:54], 16)

print("ticks=" + str(move_ticks))
print("steps=" + str(move_steps))
print("acc1=" + str(move_accumulator1))
print("rate1=" + str(move_rate1))
print("pos1=" + str(move_position1))
'''