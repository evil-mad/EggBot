import sys
from plotink import ebb_serial

def query(port_name, cmd):
    ''' Simple serial query function '''
    port_response = ''
    if port_name is not None and cmd is not None:
        try:
            port_name.write(cmd.encode('ascii'))
            port_response = port_name.readline()
            n_retry_count = 0
            while len(port_response) == 0 and n_retry_count < 100:
                port_response = port_name.readline()
                n_retry_count += 1
        except:
            print("Error reading serial data.")
    return port_response

the_port = ebb_serial.openPort()
if the_port is None:
    print("failed to connect")
    sys.exit() # end script

the_port.reset_input_buffer()

command_list = [
"SM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\nSM,1000,1000,1000\r\n",
]

response = query(the_port, 'V\r')
print(response)

for command in command_list:
    print(command + " :: " + str(query(the_port, command + '\r')).strip())

the_port.close()