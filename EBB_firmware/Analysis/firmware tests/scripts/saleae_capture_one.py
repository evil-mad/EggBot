# Generalized EBB Saleae capture function
#
# Pass in a filename to store the output, and a duration in milliseconds

from saleae import automation
import os
import os.path
import time
from ebb_serial_utility import query

# Pass in an EBB command a file path name and a time and this function
# will start the capture (with length of the time), send the EBB command,
# and then save off the resulting Saleae capture file (.sal) as well as the
# .csv file of the bytes on the debug serial port into the capture_dir_name 
# directory

def capture_command(EBB_command : str, capture_dir_path : str, capture_time : float, the_port):
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
        
        # This one is for Saleae 16
        # TODO: Can we auto detect which we are talking to and auto-switch between these?
        device_configuration = automation.LogicDeviceConfiguration(
            enabled_digital_channels=[0, 1, 2, 3, 4, 5, 6, 7, 8],
            digital_sample_rate=32_000_000,
        )
        # This one is for Saleae 16 Pro
        #device_configuration = automation.LogicDeviceConfiguration(
        #    enabled_digital_channels=[0, 1, 2, 3, 4, 5, 6, 7, 8],
        #    digital_sample_rate=50_000_000,
        #)

        # Record 5 seconds of data before stopping the capture
        capture_configuration = automation.CaptureConfiguration(
            capture_mode=automation.TimedCaptureMode(duration_seconds=capture_time)
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
            # But wait a bit first in case there is a delay starting up the capture
            time.sleep(0.2)
            response = str(query(the_port, EBB_command + '\r'))
            # TODO: Confirm that response was "OK\r\n", print error and actual response if not
            #print(last_command + " :: " + response.strip())

            # Wait until the capture has finished
            capture.wait()

            analyzers = []

            # Add an analyzer to the capture
            async_analyzer = capture.add_analyzer('Async Serial', label=f'ISR Serial', settings={
                'Input Channel': 7,
                'Bit Rate (Bits/s)': 4000000,
            })
            analyzers.append(automation.DataTableExportConfiguration(async_analyzer, automation.RadixType.HEXADECIMAL))

            # Create a directory name to store our output files in
            os.makedirs(capture_dir_path, exist_ok = True)

            # Export analyzer data to a CSV file
            analyzer_export_filepath = os.path.join(capture_dir_path, 'debug_serial.csv')
            capture.export_data_table(
                filepath=analyzer_export_filepath,
                analyzers=analyzers
            )

            # Export raw digital data to a CSV file 
            capture.export_raw_data_csv(directory=capture_dir_path, digital_channels=[0, 1, 2, 3, 4, 5, 6, 7, 8])

            # Save the capture to a file
            capture_filepath = os.path.join(capture_dir_path, 'capture.sal')
            capture.save_capture(filepath=capture_filepath)

            # Is this param.csv file really necessary? What is need?
            #       Save off text file with EBB command and parameters
            #with open(os.path.join(capture_dir_path, 'param.csv'), 'w') as param_file:
            #    param_file.write(EBB_version.decode() + '\n')
            #    param_file.write(EBB_command + '\n')
            #    param_file.write(capture_dir_path + '\n')
            #    param_file.write(str(capture_time) + '\n')
