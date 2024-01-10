#!/usr/bin/env python
# -*- encoding: utf-8 -#-

from saleae import automation
import os
import os.path
from datetime import datetime
import sys
import time

from pyaxidraw import axidraw
from plotink import ebb_motion
from plotink import ebb_serial



def query(port_name, cmd):
    if port_name is not None and cmd is not None:
        response = ''
        try:
#             port_name.write(cmd.encode('ascii'))
            port_name.write(cmd.encode('ascii'))
            response = port_name.readline()
            n_retry_count = 0
            while len(response) == 0 and n_retry_count < 100:
                # get new response to replace null response if necessary
                response = port_name.readline()
                n_retry_count += 1
        except:
            print("Error reading serial data.")
        return response

def block(ad_ref, timeout_ms=None):
    '''
    Interactive context: Wait until all motion control commands have finished, or an
    an optional timeout occurs.

    Polls the EBB immediately and then every 10 ms thereafter until (1) Neither motor is
    currently in motion and (2) there is no queued motion control command.

    A value for timeout_ms, gives the maximum duration to wait in milliseconds.

    Returns True if the motion queue is empty, and False if timed out.

    Requires EBB version v2.6.2 or newer
    '''

    if timeout_ms is None:
        time_left = 60000 # Default 60 second timeout.
    else:
        time_left = timeout_ms

    while True:
        qg_val = bytes.fromhex(ebb_serial.query(ad_ref.plot_status.port, 'QG\r').strip())
        motion = qg_val[0] & (15).to_bytes(1, byteorder='big')[0] # Motion queue bits
        if motion == 0:
            return True
        if time_left <= 0:
            return False    # Timed out
        if time_left < 10:
            time.sleep(time_left / 1000) # Use up remaining time
            time_left = 0
        else:
            time.sleep(0.01) # Sleep 10 ms
            if timeout_ms is not None:
                time_left -= 10


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


# Note: 
command_list = [
"SM,100,100,0",
"SM,100,-100,0",
"SM,100,0,100",
"SM,100,0,-100",
"SM,100,100,100",
"SM,100,-100,-100",
"XM,100,100,0",
"XM,100,-100,0",
"XM,100,0,100",
"XM,100,0,-100",
"XM,100,100,100",
"XM,100,-100,-100",
]

print("connected")

response = query(the_port, 'V\r')
print(response)

last_command = ""

response = str(query(the_port, "CU,250,1" + '\r'))
print(last_command + " :: " + response.strip())
response = str(query(the_port, "CU,251,1" + '\r'))
print(last_command + " :: " + response.strip())


# Connect to the running Logic 2 Application on port `10430`.
# Alternatively you can use automation.Manager.launch() to launch a new Logic 2 process - see
# the API documentation for more details.
# Using the `with` statement will automatically call manager.close() when exiting the scope. If you
# want to use `automation.Manager` outside of a `with` block, you will need to call `manager.close()` manually.
with automation.Manager.connect(port=10430) as manager:

    # Configure the capturing device to record on digital channels 0, 1, 2, and 3,
    # with a sampling rate of 10 MSa/s, and a logic level of 3.3V.
    # The settings chosen here will depend on your device's capabilities and what
    # you can configure in the Logic 2 UI.
    device_configuration = automation.LogicDeviceConfiguration(
        enabled_digital_channels=[0, 1, 2, 3, 4, 5, 6, 7, 8],
        digital_sample_rate=32_000_000,
    )

    # Record 5 seconds of data before stopping the capture
    capture_configuration = automation.CaptureConfiguration(
        capture_mode=automation.TimedCaptureMode(duration_seconds=2.0)
    )

    # Start a capture - the capture will be automatically closed when leaving the `with` block
    # Note: The serial number 'F4241' is for the Logic Pro 16 demo device.
    #       To use a real device, you can:
    #         1. Omit the `device_id` argument. Logic 2 will choose the first real (non-simulated) device.
    #         2. Use the serial number for your device. See the "Finding the Serial Number
    #            of a Device" section for information on finding your device's serial number.
    with manager.start_capture(
            device_configuration=device_configuration,
            capture_configuration=capture_configuration) as capture:

        # The capture has started. We now need to send our EBB command:
        response = str(query(the_port, "SM,1000,1234,567" + '\r'))
        print(last_command + " :: " + response.strip())

        # Wait until the capture has finished
        # This will take about 5 seconds because we are using a timed capture mode
        capture.wait()

        analyzers = []

        # Add an analyzer to the capture
        async_analyzer = capture.add_analyzer('Async Serial', label=f'ISR Serial', settings={
            'Input Channel': 7,
            'Bit Rate (Bits/s)': 3000000,
        })
        analyzers.append(automation.DataTableExportConfiguration(async_analyzer, automation.RadixType.HEXADECIMAL))

        # Store output in a timestamped directory
        output_dir = os.path.join(os.getcwd(), f'output-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')
        os.makedirs(output_dir)

        # Export analyzer data to a CSV file
        analyzer_export_filepath = os.path.join(output_dir, 'async_serial_export.csv')
        capture.export_data_table(
            filepath=analyzer_export_filepath,
            analyzers=analyzers
        )

        # Export raw digital data to a CSV file
        capture.export_raw_data_csv(directory=output_dir, digital_channels=[0, 1, 2, 3, 4, 5, 6, 7])

        # Finally, save the capture to a file
        capture_filepath = os.path.join(output_dir, 'example_capture.sal')
        capture.save_capture(filepath=capture_filepath)





#for command in command_list:
#    #block(ad)
#    response = str(query(the_port, command + '\r'))
#    print(last_command + " :: " + response.strip())
#    last_command = command
    
print(last_command + " :: ")


ad.disconnect()             # Close serial port to AxiDraw



