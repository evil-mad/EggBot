'''
Simulating case of CM, dest_x, dest_y, center_x, center_y, StepFrequency, dir

Based on Masory and Koren (1982) with "Taylor" method.

Where all of dest_x, dest_y, center_x and center_y are relative to the initial position and direction is 0 for CCW and 1 for CW.

    StepFrequency has the same units as for http://evil-mad.github.io/EggBot/ebb.html#HM

SM works as follows:
    - Limit check: Duration != 0 ms.
    - Limit check: Duration < 16777215 ms
    - Limit check: Step_count_1 < 16777215
    - Limit check: Step_count_1 / duration_ms  < 25 000
    - Limit check: duration_ms / 13011  > Step_count_1 -> Rate > 1.31 Hz
    - Limit check: Step_count_2 < 16777215
    - Limit check: Step_count_2 / duration_ms  < 25 000
    - Limit check: duration_ms / 13011  > Step_count_2 -> Rate > 1.31 Hz

    process_simple_motor_move()
        -Apply accumulator clearing bits
        - Enable motors
        - Set direction bits

        if 0 < axis1 steps < 0x1FFFF  ( 131071 ):
            Rate1 = ticks_per_ms * duration_ms
            Intervals = Step_count_1 << 15 / Rate1

            (Intervals here becomes the "rate" factor in the ISR, which
                is what gets added to the accumulator each ISR.)
    
            if duration_ms > 30:
                # Only do this for long moves, to reduce number of divisions
                Rate2 = (Step_count_1 << 15) % gTmpRate1;
                remainder = (Rate2 << 16) / Rate1;
        else:
            Intervals = ( (Step_count_1 << 15)/duration_ms ) / ticks_per_ms
        if (Intervals > 0x8000):
            error; set intervals to 0x8000 (32768)

            # With steps 16777215 (512 * 32768), duration 32768 ms:
            # Intervals = ( (Step_count_1 << 15)/duration_ms ) / ticks_per_ms
            # = ((16777215 * 32768) / 32768) / 25000 = 671

        if (Intervals ==> 0):
            error; set intervals to 1.

        if duration_ms > 30:
            Intervals = Intervals << 16 + remainder
        else
            Intervals = Intervals << 16

        if Intervals >= 0x7FFFFFFF,
            Intervals = 0x7FFFFFFF

        gMoveTemp.Rate[0].value = Intervals;
        gMoveTemp.Steps[0] = Step_count_1;
        gMoveTemp.Accel[0] = 0;

        ^^^ Repeat all that for axis 2.

        gMoveTemp.Rate[1].value = gTmpIntervals;
        gMoveTemp.Steps[1] = gTmpSteps2;
        gMoveTemp.Accel[1] = 0;

    In high_ISR():

    if (bittst(CurrentCommand.Command, COMMAND_SM_XM_HM_MOVE_BIT_NUM))

        Output direction bits
    
        Motor 1: 
            if there are steps left to take  ( if (bittstzero(AxisActive[0])) )
                accumulator += rate
                if MSB of accumulator is set:
                    Clear MSB
                    take step
                    CurrentCommand.Steps[0]--

                    if CurrentCommand.Steps[0] == 0:
                        Mark no steps left to take. (bitclrzero(AxisActive[0]))

        Repeat for motor 2.

        If neither axis is still active:
            Mark AllDone
            goto outputbits


FOR CM move:

    Begin with StepFreq, if not 0,  in range 1 - 25000.
    Begin with Radius in range 5 - 32767.
    Further restrict distance to destination to range [10, 32767] steps

    - For calculating radius, need distance to center < 32767
    - Distance from center to final position should also be < 32767

    Initial processing:
    - Calculate center position & offset.

    - Calculate angle 1/alpha factor
    - Calculate multiplicative factors for segment steps
    - Counter for number of segments segments_left = N
    - Initialize XY values that will be relative to center.

    - Calculate velocity factor & scale to 16 bits


    Calculate velocity scale factor K as follows:
        # VScaleK = (StepFreq * 2^31) / (25000 * radius * alpha)
        #         = (StepFreq * 2^31) / (3125 * 2^3 * radius * alpha)
        #         = (StepFreq * 2^28) / (3125 * radius * alpha)

    bits_left (int8) = 28 # Remaining bits left to shift left.
    bits_left += alpha_exponent # May be up to 6.

    Note that scaling of alpha with radius is alpha ~ sqrt(8/R);
        (This is not exact, as we pick only 1/2^n values for alpha.)



    Still need to find StepFreq / (25000 * radius)

    if (StepFreq >= 256):   # StepFreq is in range 256 - 25000
        StepFreq << 17
        bits_left -= 17
        # StepFreq now in range 3.35E7 - 3.27E9 (Almost 2^25 - Almost 2^32)
    else:                   # StepFreq is in range 1 - 255
        StepFreq << 24
        bits_left -= 24
        # StepFreq now in range 1.67E7 - 4.27E9 (2^24 - Almost 2^32)

    Denom (uint32) = 3125 * Radius
        # Denom now in range 1.67E7 - 4.27E9 (2^24 - Almost 2^32)
    if (Radius >= 256):     # Radius is in range 256 - 32767
        Denom >> 11
        bits_left -= 11
        # Denom now in range 3906 - 50000 (Almost 2^12 - Almost 2^16)
    else:                   # Radius is in range 5 - 255
        Denom >> 1
        bits_left -= 1
        # Denom now in range 7182 - 39843 (Almost 2^13 - Almost 2^16)

    VScaleK (uint32) = StepFreq / Denom
    # Largest possible VScaleK: 4.27E9 / 7182 = 547643 (almost 2^20)
    # Smallest possible VScaleK: 1.67E7 / 50000 = 334 (almost 2^9)

    if VScaleK > 2^16:
        VScaleK >> 4
        bits_left += 4
    # VScaleK value range is now 334 - 37158, all < 2^16.

    Next up: List of test cases
            Simulate velocity at each step
            Find initial center from radius



    In ISR
    - Work like SM command
    - If neither axis is still active:
        if segments_left == 0:
            Mark AllDone
        if segments_left == 1:
            Destination is _final position_
        else:
            Calculate destination from algorithm


Next steps:
    - Calculate center position from radius from start, end.
    - OR, if using IJ type notation, calculate angle subtended
    - Need the angle *anyway* to find number of steps.

    - Handle case of last move, getting to final position.
        - How do we detect it?
    - Work through full ISR including increments to accumulator (maybe)
    - Present pseudocode
    - Clever solution for two 16-bit multiplication operations:
        - Calculate dx
        - Calculate dy
        - calculate vx
        - calculate vy
    - Only calculate next values if this is not the last segment of the move.
    - Can we guarantee that no move is shorter than 5 ISRs? Seems like with R = 5, 
        typical segments are indeed about 6 steps long. However, a move of (3,-3) could
        theoretically happen in 3 loops.
    - Add test in our loops to find the shortest distance. Can any be less than 3?





Notes:
    - Algorithm works best for curves < 1/4 circle.
    - Does not work as well for R <= 20 steps. BUT IT WORKS PRETTY WELL.
    - Probably best to advise replacing with a straight SM for moves with < 10 step
        radius.
    - Get great results with a floating point X, Y value.
    - Not great with X, Y rounded into motor steps.
    - Is there any way to use our deeper position knowledge (accumulator values)
        as part of the calculation? 
        - Probably _not_ -- and not a great approach. The final accumulator value
            is kind of arbitrary in a sense.
    - Can we pick out some additional bits of precision? Suppose that the moves
        are made with R = ±32768 max (± 2^15) -- for speed.  We could instead
        limit curves to ±2^11 (±2048), with 4 bits of sub-step precision.
    - With 24-bit local positions, could have 20 bits of position precision

'''

import math
from struct import *

import matplotlib.pyplot as plt # TODO REMOVE PRIOR TO MERGE

ACCUM_MAX = 2147483648  # 2^31
ISR_MAX = 250000 # Maximum cycle count for calculating movements

# Test Cases: 
# step_freq, center_x, center_y, x_dest, y_dest, direction: 0 CW, 1 CCW

test_cases = [


[1000, 100, 0, 100, -100, 1],
[1000, 0, 100, 100, 100, 1],
[1000, -100, 0, -100, 100, 1],
[1000, 0, -100, -100, -100, 1],

[1000, 100, 0, 100, 100, 1],
[1000, 0, 100, -100, 100, 1],
[1000, -100, 0, -100, -100, 1],
[1000, 0, -100, 100, -100, 1],

[1000, 5, 5, 10, 0, 0],
[1000, -5, 0, -10, 0, 1],
[1000, 50, 50, 100, 0, 0],
[1000, 0, 500, 500, 500, 1],
[1000, 1000, 0, 1707, 707, 0],
[1000, 1000, 0, 1707, 707, 1],


# [1, 10], 
# [1, 20], 
# [255, 5],
# [25000, 5],
# [1, 50], 
# [255, 50],
# [25000, 50],
# [1, 32767], 
# [255, 32767],
# [25000, 32767],
]

# step_freq, radius, x_dest, y_dest, direction

verbose = True
print_all_steps = False
print_all_calc = False
plot_graph = True

error_list = []
overflows = False 

for case in test_cases:
    step_freq = case[0] # Initial step frequency; uint16
    center_x = case[1] # int16
    center_y = case[2] # int16
    dest_x = case[3] # int16
    dest_y = case[4] # int16
    direction = case[5] # 0 or 1

    x_t = - center_x # int32
    y_t = - center_y # int32
    x_f = dest_x - center_x # int16
    y_f = dest_y - center_y # int16

    radius = math.floor(math.sqrt(center_x * center_x + center_y * center_y))

    segment_count = 0 # uint8
    print(f"\nNew test case:")

    print(f"StepFreq = {step_freq}, Radius = {radius}")
    print(f"Center position: ({center_x}, {center_y})")
    print(f"Destination wrt center: ({x_f}, {y_f})")

    
    ''' Find alpha value.
        Alpha is 2^-m, where m is the smallest integer
            such that  2^(2 m + 3) >= radius
    '''
    
    alpha_factor = 0
    while True: 
        # temp = 2 * alpha_factor + 3 
        temp2 = 2 ** (2 * alpha_factor + 3) 
        if temp2 >= radius:
            print(f"Found alpha = 1/{2 ** alpha_factor}; alpha_factor = {alpha_factor}")
            break
        alpha_factor += 1


    # Alpha is used for floats & plotting only.
    # alpha = 1/(2 ** alpha_factor) # Angle taken by individual subsegments
    # For plotting
    segments_est_tot = 200
    # segments_left = int(math.pi / (2 * alpha))

    # Typical segment length: radius * alpha = radius >> alpha_factor
    # 1.5 * typical segment length: # I think it can be a uint8.
    #   radius >> alpha_factor + radius >> (alpha_factor + 1)

    typ_seg = (radius >> alpha_factor) 
    typ_seg_long = typ_seg + (radius >> (alpha_factor + 1))

    print(f"typ_seg = {typ_seg}, typ_seg_long = {typ_seg_long}")

    if abs(x_t - x_f) < typ_seg_long and abs(y_t - y_f) < typ_seg_long:
        print(f"SHORT MOVE - Handle as SM / process_simple_motor_move()")
        continue











    bits_left = 28
    bits_left += alpha_factor

    # Precompute velocity scaling factor VScaleK:

    StepFreq = step_freq # uint32
    if (StepFreq >= 256):   # StepFreq is in range 256 - 25000
        StepFreq = StepFreq << 17
        bits_left -= 17
        # StepFreq now in range 3.35E7 - 3.27E9 (Almost 2^25 - Almost 2^32)
        print(f"StepFreq: {StepFreq}. bits_left = {bits_left} after shifting << 17")

    else:                   # StepFreq is in range 1 - 255
        StepFreq = StepFreq << 24
        bits_left -= 24
        # StepFreq now in range 1.67E7 - 4.27E9 (2^24 - Almost 2^32)
        print(f"StepFreq: {StepFreq}. bits_left = {bits_left} after shifting << 24")

    denom = 3125 * radius # uint32
        # Denom now in range 1.67E7 - 4.27E9 (2^24 - Almost 2^32)
    if (radius >= 256):     # Radius is in range 256 - 32767
        denom = denom >> 11
        bits_left -= 11
        # Denom now in range 3906 - 50000 (Almost 2^12 - Almost 2^16)

        print(f"denom: {denom}. bits_left = {bits_left} after shifting >> 11")

    else:                   # Radius is in range 1 - 255
        denom = denom >> 1
        bits_left -= 1
        # Denom now in range 1562 - 39843 (Almost 2^11 - Almost 2^16)
    
        print(f"denom: {denom}. bits_left = {bits_left} after shifting >> 1")

    VScaleK = int(StepFreq / denom) # The big division operator!
    # Largest possible VScaleK: 4.27E9 / 1562 = 2.733E6 (almost 2^22)
    # Smallest possible VScaleK: 1.67E7 / 50000 = 334 (almost 2^9)

    if VScaleK > 65536:
        VScaleK = VScaleK >> 4
        bits_left += 4
    # VScaleK value range is now 334 - 42,703, all < 2^16.
    print(f"VScaleK: {VScaleK}. bits_left = {bits_left}.")


    # For development only:
    if bits_left > 0:
        VScaleK_nobits = VScaleK << bits_left
    elif bits_left < 0:
        VScaleK_nobits = VScaleK >> -bits_left
    else:
        VScaleK_nobits = VScaleK

    print(f"Applying scale bits: VScaleK = {VScaleK_nobits:.0f}")
    alpha = 1/(2 ** alpha_factor) # Used in float computation only
    VScaleK_float = (step_freq * 2147483648) / (25000 * radius * alpha)
    # print(f"Floating point expected value: {VScaleK_float:.0f}")
    # print(f"Ratio:                         {VScaleK_float / VScaleK_nobits:.6f}")


   # Charting & keeping track of position errors:
    err_max = 0
    exes= [x_t] 
    whys = [y_t]
    vees = [radius * step_freq / 25000]
    tees = [-radius]

    x_pos_last = x_t # int16 to Store last (16-bit) position value
    y_pos_last = y_t # int16

    # Scale X, Y values up by 65536. The actual positions are 16-bit signed, but
    #   we need additional resolution to avoid adding rounding errors as we
    #   do compute each new position from the last position.

    x_t = x_t << 16
    y_t = y_t << 16

    # This concludes the "precomputation" stage


    # Calculate steps & velocity for first subsegment
    # This is the same set of calcs that we will do in the ISR for computing
    #   subsequent subsegments


    segment_count += 1 # For velocity chart only

    # Nominal intent:
    # d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) - (y_t >> alpha_factor)
    # d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) + (x_t >> alpha_factor)
    # BUT, right shifts with negative numbers aren't "clean" division

    # There is probably some way to simplify this
    # if direction:
    #     if x_t < 0 and y_t < 0:
    #         x_s = -x_t
    #         y_s = -y_t
    #         d_x = ( x_s >> (2 * alpha_factor + 1)) - (y_s >> alpha_factor)
    #         d_y = ( y_s >> (2 * alpha_factor + 1)) + (x_s >> alpha_factor)
    #     elif y_t < 0:
    #         y_s = -y_t
    #         d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) - (y_s >> alpha_factor)
    #         d_y = ( y_s >> (2 * alpha_factor + 1)) - (x_t >> alpha_factor)
    #     elif x_t < 0:
    #         x_s = -x_t
    #         d_x = ( x_s >> (2 * alpha_factor + 1)) + (y_t >> alpha_factor)
    #         d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) + (x_s >> alpha_factor)
    #     else:
    #         d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) + (y_t >> alpha_factor)
    #         d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) - (x_t >> alpha_factor)
    # else:
    #     if x_t < 0 and y_t < 0:
    #         x_s = -x_t
    #         y_s = -y_t
    #         d_x = ( x_s >> (2 * alpha_factor + 1)) + (y_s >> alpha_factor)
    #         d_y = ( y_s >> (2 * alpha_factor + 1)) - (x_s >> alpha_factor)
    #     elif y_t < 0:
    #         y_s = -y_t
    #         d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) + (y_s >> alpha_factor)
    #         d_y = ( y_s >> (2 * alpha_factor + 1)) + (x_t >> alpha_factor)
    #     elif x_t < 0:
    #         x_s = -x_t
    #         d_x = ( x_s >> (2 * alpha_factor + 1)) - (y_t >> alpha_factor)
    #         d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) - (x_s >> alpha_factor)
    #     else:
    #         d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) - (y_t >> alpha_factor)
    #         d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) + (x_t >> alpha_factor)

    alpha_shift = alpha_factor + alpha_factor + 1

    if x_t < 0:
        x_1 = ( - x_t >> alpha_shift) 
        x_2 = -( - x_t >> alpha_factor) 
    else:
        x_1 = -( x_t >> alpha_shift) 
        x_2 = ( x_t >> alpha_factor) 

    if y_t < 0:
        y_1 = ( - y_t >> alpha_shift) 
        y_2 = -( - y_t >> alpha_factor) 
    else:
        y_1 = -( y_t >> alpha_shift) 
        y_2 = ( y_t >> alpha_factor) 

    if direction:
        d_x = x_1 + y_2
        d_y = y_1 - x_2
    else:
        d_x = x_1 - y_2
        d_y = y_1 + x_2






    print(f" Old Pos: ({x_pos_last:.3f}, {y_pos_last:.3f}).")

    # if direction:
    #     d_x = -d_x
    #     d_y = -d_y

    print(f" Delta: ({d_x:.3f}, {d_y:.3f}).")

    # Use floats to compute midpoint for error checking (in development only):
    x_mid = (x_t + (d_x / 2.0)) / 65536.0
    y_mid = (y_t + (d_y / 2.0)) / 65536.0

    err = math.sqrt(x_mid * x_mid + y_mid * y_mid) - radius
    if abs(err) > err_max:
        err_max = abs(err)

    # Update 32-bit position values:
    x_t += d_x
    y_t += d_y


    # Find new 16-bit position value. 
    # In firmware, take two high bytes of position; do not actually shift/divide.

    x_t_16 = unpack("@hh", pack("@i", x_t))[1]
    y_t_16 = unpack("@hh", pack("@i", y_t))[1]

    # Find Step count and directions.
    x_steps = x_t_16 - x_pos_last
    y_steps = y_t_16 - y_pos_last

    # print(f" Steps: ({x_steps}, {y_steps})")


    if x_steps < 0:
        stepdir_x = 0 # Set direction bits
        x_stepcount = -x_steps
    else:
        stepdir_x = 1 # Set direction bits
        x_stepcount = x_steps
    if y_steps < 0:
        stepdir_y = 0 # Set direction bits
        y_stepcount = -y_steps
    else:
        stepdir_y = 1 # Set direction bits
        y_stepcount = y_steps

    # Rate factors: v_x, v_y : 32-bit unsigned
    if bits_left >= 0:
        v_x = (x_steps * VScaleK) << bits_left
        v_y = (y_steps * VScaleK) << bits_left
    else:
        v_x = (x_steps * VScaleK) >> -bits_left
        v_y = (y_steps * VScaleK) >> -bits_left

    # Possibly add check to ensure that v_x, v_x < 2^31.

    x_pos_last = x_t_16
    y_pos_last = y_t_16

    # Use floats to compute radius for error checking (in development only):
    x_s = x_t / 65536
    y_s = y_t / 65536

    err = math.sqrt(x_s * x_s + y_s * y_s) - radius
    if abs(err) > err_max:
        err_max = abs(err)

    exes.append(x_t_16)
    whys.append(y_t_16)

    rate_avg = math.sqrt(v_x * v_x + v_y * v_y)
    vees.append(radius * rate_avg /2147483648)

    tees.append(radius * ((2 * segment_count / segments_est_tot) - 1))

    last_seg = False

    while True:     # Iterate through subsegments
        segment_count += 1 # For velocity chart only

        if segment_count > 105:
            break

        print(f" Old Pos: ({x_pos_last:.3f}, {y_pos_last:.3f}).")


        if abs(x_f - x_pos_last) < typ_seg_long and abs(y_f - y_pos_last) < typ_seg_long:
            # Last segment of move!
            last_seg = True

            print(f" x_pos_last, y_pos_last: ({x_pos_last:.3f}, {y_pos_last:.3f}) ")
            print(f" x_f, y_f: ({x_f:.3f}, {y_f:.3f}) ")

            x_steps = x_f - x_pos_last
            y_steps = y_f - y_pos_last

            print(f" x_steps, y_steps: ({x_steps:.3f}, {y_steps:.3f}) ")




        # Nominal intent:
        # d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) - (y_t >> alpha_factor)
        # d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) + (x_t >> alpha_factor)
        # BUT, right shifts with negative numbers aren't "clean" division
    
    #     if x_t < 0 and y_t < 0:
    #         x_s = -x_t
    #         y_s = -y_t
    #         d_x = ( x_s >> (2 * alpha_factor + 1)) + (y_s >> alpha_factor)
    #         d_y = ( y_s >> (2 * alpha_factor + 1)) - (x_s >> alpha_factor)
    # 
    #     elif y_t < 0:
    #         y_s = -y_t
    #         d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) + (y_s >> alpha_factor)
    #         d_y = ( y_s >> (2 * alpha_factor + 1)) + (x_t >> alpha_factor)
    # 
    #     elif x_t < 0:
    #         x_s = -x_t
    #         d_x = ( x_s >> (2 * alpha_factor + 1)) - (y_t >> alpha_factor)
    #         d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) - (x_s >> alpha_factor)
    #     else:
    #         d_x = -1 * ( x_t >> (2 * alpha_factor + 1)) - (y_t >> alpha_factor)
    #         d_y = -1 * ( y_t >> (2 * alpha_factor + 1)) + (x_t >> alpha_factor)





        alpha_shift = alpha_factor + alpha_factor + 1

        if x_t < 0:
            x_1 = ( - x_t >> alpha_shift) 
            x_2 = -( - x_t >> alpha_factor) 
        else:
            x_1 = -( x_t >> alpha_shift) 
            x_2 = ( x_t >> alpha_factor) 

        if y_t < 0:
            y_1 = ( - y_t >> alpha_shift) 
            y_2 = -( - y_t >> alpha_factor) 
        else:
            y_1 = -( y_t >> alpha_shift) 
            y_2 = ( y_t >> alpha_factor) 

        if direction:
            d_x = x_1 + y_2
            d_y = y_1 - x_2
        else:
            d_x = x_1 - y_2
            d_y = y_1 + x_2





        '''
        if direction:
            if x_t < 0 and y_t < 0:
                x_s = -x_t
                y_s = -y_t
                d_x = ( x_s >> alpha_shift) - (y_s >> alpha_factor)
                d_y = ( y_s >> alpha_shift) + (x_s >> alpha_factor)
            elif y_t < 0:
                y_s = -y_t
                d_x = -1 * ( x_t >> alpha_shift) - (y_s >> alpha_factor)
                d_y = ( y_s >> alpha_shift) - (x_t >> alpha_factor)
            elif x_t < 0:
                x_s = -x_t
                d_x = ( x_s >> alpha_shift) + (y_t >> alpha_factor)
                d_y = -1 * ( y_t >> alpha_shift) + (x_s >> alpha_factor)
            else:
                d_x = -1 * ( x_t >> alpha_shift) + (y_t >> alpha_factor)
                d_y = -1 * ( y_t >> alpha_shift) - (x_t >> alpha_factor)
        else:
            if x_t < 0 and y_t < 0:
                x_s = -x_t
                y_s = -y_t
                d_x = ( x_s >> alpha_shift) + (y_s >> alpha_factor)
                d_y = ( y_s >> alpha_shift) - (x_s >> alpha_factor)
            elif y_t < 0:
                y_s = -y_t
                d_x = -1 * ( x_t >> alpha_shift) + (y_s >> alpha_factor)
                d_y = ( y_s >> alpha_shift) + (x_t >> alpha_factor)
            elif x_t < 0:
                x_s = -x_t
                d_x = ( x_s >> alpha_shift) - (y_t >> alpha_factor)
                d_y = -1 * ( y_t >> alpha_shift) - (x_s >> alpha_factor)
            else:
                d_x = -1 * ( x_t >> alpha_shift) - (y_t >> alpha_factor)
                d_y = -1 * ( y_t >> alpha_shift) + (x_t >> alpha_factor)
        '''



        # print(f" Delta: ({d_x:.3f}, {d_y:.3f}) -- in loop")

        # Use floats to compute midpoint for error checking (in development only):
        x_mid = (x_t + (d_x / 2.0)) / 65536.0
        y_mid = (y_t + (d_y / 2.0)) / 65536.0
    
        err = math.sqrt(x_mid * x_mid + y_mid * y_mid) - radius
        if abs(err) > err_max:
            err_max = abs(err)


    
        # Update 32-bit position values:
        x_t += d_x
        y_t += d_y

        # if direction:
        #     x_t += -d_x
        #     y_t += -d_y
        # else:
        #     x_t += d_x
        #     y_t += d_y



        # Find new 16-bit position value. 
        # In firmware, take two high bytes of position; do not actually shift/divide.
        x_t_16 = unpack("@hh", pack("@i", x_t))[1]
        y_t_16 = unpack("@hh", pack("@i", y_t))[1]
    
    
        # Find Step count and directions. 
        x_steps = x_t_16 - x_pos_last
        y_steps = y_t_16 - y_pos_last


        # print(f" V: ({v_x:.3f}, {v_y:.3f}). Scaled: ({v_x / 2147483648:.3f}, {v_y / 2147483648:.3f}) ")
        # print(f" V Scaled: ({v_x / 2147483648:.3f}, {v_y / 2147483648:.3f}) ")






        if last_seg:
            x_steps_final = x_f - x_pos_last
            y_steps_final = y_f - y_pos_last

            # Fine-tune step counts _for computing velocity_

            if x_steps_final > 0:
                if x_steps > 0:
                    x_steps = x_steps_final
                else:
                    x_steps = 1
            elif x_steps_final < 0:
                if x_steps < 0:
                    x_steps = x_steps_final
                else:
                    x_steps = -1
            else:
                x_steps = 0

            if y_steps_final > 0:
                if y_steps > 0:
                    y_steps = y_steps_final
                else:
                    y_steps = 1
            elif x_steps_final < 0:
                if y_steps < 0:
                    y_steps = y_steps_final
                else:
                    y_steps = -1
            else:
                y_steps = 0


        # Rate factors: v_x, v_y : 32-bit unsigned
        if bits_left >= 0:
            v_x = (x_steps * VScaleK) << bits_left
            v_y = (y_steps * VScaleK) << bits_left
        else:
            v_x = (x_steps * VScaleK) >> -bits_left
            v_y = (y_steps * VScaleK) >> -bits_left

        if last_seg:
            x_steps = x_steps_final
            y_steps = y_steps_final


        print(f" Steps: ({x_steps:.3f}, {y_steps:.3f}).")

    


    
        # print(f" Steps: ({x_steps}, {y_steps})")
    
        if x_steps < 0:
            stepdir_x = 0 # Set direction bits
            x_stepcount = -x_steps
        else:
            stepdir_x = 1 # Set direction bits
            x_stepcount = x_steps
        if y_steps < 0:
            stepdir_y = 0 # Set direction bits
            y_stepcount = -y_steps
        else:
            stepdir_y = 1 # Set direction bits
            y_stepcount = y_steps





        # Use floats to compute radius for error checking (in development only):
        x_s = x_t / 65536
        y_s = y_t / 65536
    
        err = math.sqrt(x_s * x_s + y_s * y_s) - radius
        if abs(err) > err_max:
            err_max = abs(err)


        exes.append(x_steps + x_pos_last)
        whys.append(y_steps + y_pos_last)

        rate_avg = math.sqrt(v_x * v_x + v_y * v_y)
        vees.append(radius * rate_avg /2147483648)
        tees.append(radius * ((2 * segment_count / segments_est_tot) - 1))

        if print_all_steps:
            print(f" Delta: ({d_x:.3f}, {d_y:.3f}). X: ({x_s:.3f}, {y_s:.3f}) ")

        # print(f" Steps: ({x_steps:.3f}, {y_steps:.3f}) ")


        x_pos_last = x_t_16
        y_pos_last = y_t_16


        if last_seg:
            break

    
    
    print(f" Max err: {err_max:.3f} ")
    
    
    if plot_graph:  # PLOT DATA FOR DEV USE ONLY, best for one path only. ;)

        r_exes = [radius]
        r_whys = [0]
        for angle_step in range(1000):
            r_exes.append(radius * math.cos(math.tau * angle_step / 1000))
            r_whys.append(radius * math.sin(math.tau * angle_step / 1000))
    
        plt.style.use('_mpl-gallery')
        plt.figure(figsize=(6, 4), layout='tight')
        plt.margins(x=3, y=None, tight=True)
        plt.plot(exes, whys, 'o-',  color='purple', label='m1', markersize='2')
        plt.plot(r_exes, r_whys, '-',  color='green', label='m1', markersize='2', linewidth=0.5)
    
        plt.plot(tees, vees, '-',  color='blue', label='m1', markersize='2', linewidth=0.5)
    
    
        plt.xlim(-radius * 1.05, radius * 1.05)
    
        # plt.scatter(exes,whys,s=1)
        # plt.xlim([times[0] - times[-1] / 100,
        #     times[-1] +  times[-1] / 100])
        # plt.show()

        plt.show(block=False)
        input("Hit Enter To Close")
        plt.close()



'''
print("\n\nCommand log follows:\n")
print(command_log)
print("\n\nResult log follows:\n")
print(result_log)

print("\n\nebb_calc.move_dist_t3 testing log follows:\n")
print(move_dist_t3_testing_log)


print("\n\nebb_calc.rate_t3_testing_log testing log follows:\n")
print(rate_t3_testing_log)


print("\n\nebb_calc.max_rate_t3_testing_log testing log follows:\n")
print(max_rate_t3_testing_log)

'''

if "no_end" in error_list:
    print("ERROR(s): Max ISRs exceeded; runaway loop likely found")
if "final_pos" in error_list:
    print("ERROR(s): Final position mismatch found")
if "final_time" in error_list:
    print("ERROR(s): Time calculation mismatch found ")
if "final_accum" in error_list:
    print("ERROR(s): Final accumulator mismatch found ")
if "final_rate" in error_list:
    print("ERROR(s): Final rate mismatch found ")
if "max_rate" in error_list:
    print("ERROR(s): Max rate mismatch found ")

print("")


if overflows:
    print(f"Some tests invalid due to rate overflows.")

if len(error_list) == 0:
    print(f"No calculation errors detected.")

# print(error_list)
