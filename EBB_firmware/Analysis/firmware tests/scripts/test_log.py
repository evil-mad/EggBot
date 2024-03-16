# A very simple holder for our 'global' test output log file and associated functionality

import os
from datetime import datetime


# global variable to hold the file pointer to our output log file
#test_log_file

# global variable which holds the path to this test run's top level artifact directory
artifact_dir = ""

def tl_init():
    global test_log_file
    global artifact_dir
    artifact_dir = os.path.join(os.getcwd(), f'../test outputs/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}/')
    os.makedirs(artifact_dir, exist_ok = True)

    # Open test log file
    test_log_file = open(artifact_dir + f'test_log.txt', "w")

def tl_close():
    global artifact_dir
    global test_log_file
    test_log_file.close()

# Function to 'print' a message
# normally goes to both command line as well as test log file
def tl_print(print_str : str, nextline : bool = True):
    global artifact_dir
    global test_log_file
    if nextline == True:
        print(print_str)
        test_log_file.write(print_str + f'\n')
    else:
        print(print_str, end='')
        test_log_file.write(print_str)

