# Utility functions for talking to an EBB over USB serial

import time
import re
from plotink import ebb_serial


def query(port_name, cmd : str, flush_first : bool = False):
    if port_name is not None and cmd is not None:
        response = ''
        try:
            if flush_first:
                response = port_name.read(port_name.in_waiting)

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
    return None

# Extract the actual version number 'X.Y.Z' from cur_ver, and return
# true if it's less than test_ver
def EBB_version_less_than(cur_ver : str, test_ver : str):

    cur_match = re.search(r" ([0-9]+)\.([0-9]+)\.([0-9]+)", cur_ver)
    test_match = re.search(r"([0-9]+)\.([0-9]+)\.([0-9]+)", test_ver)

    if cur_match and test_match:
        cur_major = int(cur_match[1])
        cur_minor = int(cur_match[2])
        cur_patch = int(cur_match[3])
        test_major = int(test_match[1])
        test_minor = int(test_match[2])
        test_patch = int(test_match[3])

        if cur_major < test_major:
            return True
        
        if cur_major == test_major:
            if cur_minor < test_minor:
                return True
            
            if cur_minor == test_minor:
                if cur_patch < test_patch:
                    return True
                
        return False

    else:
        print("Error: Version number pattern not found in version string: " + cur_ver)
        return None