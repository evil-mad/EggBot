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


with open(filename+'/digital.csv', newline='') as f:
    reader = csv.reader(f)
    try:
        for row in reader:
            if reader.line_num != 0:
                line_time = row[0]
                step1 = row[1]
                dir1 = row[2]
                step2 = row[3]
                dir2 = row[4]

                if last_step1 == '0' and step1 == '1':
                    step1_count = step1_count + 1

                if last_step2 == '0' and step2 == '1':
                    step2_count = step2_count + 1

                last_step1 = step1
                last_dir1 = dir1
                last_step2 = step2
                last_dir2 = dir2    


                #print(row)
                #if reader.line_num >= 10:
                #    sys.exit('done')

    except csv.Error as e:
        sys.exit('file {}, line {}: {}'.format(filename, reader.line_num, e))

print(step1_count)
print(step2_count)

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
