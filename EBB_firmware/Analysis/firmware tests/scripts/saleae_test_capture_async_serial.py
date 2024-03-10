from saleae import automation
import os
import os.path
from datetime import datetime
import sys
import time

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

    # Record 2 seconds of data before stopping the capture
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

        # Wait until the capture has finished
        # This will take about 5 seconds because we are using a timed capture mode
        capture.wait()

        # Add an analyzer to the capture
        async_analyzer = capture.add_analyzer('Async Serial', label=f'ISR Serial', settings={
            'Input Channel': 7,
            'Bit Rate (Bits/s)': 3000000,
        })

        # Store output in a timestamped directory
        output_dir = os.path.join(os.getcwd(), f'output-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')
        os.makedirs(output_dir)

        # Export analyzer data to a CSV file
        analyzer_export_filepath = os.path.join(output_dir, 'async_serial_export.csv')
        capture.export_data_table(
            filepath=analyzer_export_filepath,
            analyzers=[async_analyzer]
        )

        # Export raw digital data to a CSV file
        capture.export_raw_data_csv(directory=output_dir, digital_channels=[0, 1, 2, 3, 4, 5, 6, 7])

        # Finally, save the capture to a file
        capture_filepath = os.path.join(output_dir, 'example_capture.sal')
        capture.save_capture(filepath=capture_filepath)
