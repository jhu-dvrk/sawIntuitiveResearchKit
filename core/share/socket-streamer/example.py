# This is a dummy test script

# If you plan to write your own client based on this code, make sure
# you add a proper sleep in the loop to make sure you have a constant
# rate of messages.  Also, this example only works if the component
# sends message since recvfrom is blocking

import socket
import json

UDP_IP = "127.0.0.1"
UDP_PORT = 48051 # look for port in share/socket-streamer/streamerXXX.json

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind((UDP_IP, UDP_PORT))
counter = 0.0;
increment = 0.001

while True:
    # read what the component is sending
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print('received message: %s' % data)
    message = json.loads(data) # convert to Python object
    print(message)
    # send a dummy command: change direction to stay in range [-0.5, 0.5]
    if counter < -0.5 or counter > 0.5:
        increment *= -1.0
    counter += increment
    # example of write command, tested with a dVRK PSM simulated so it starts at 0, 0, 120mm, 0, 0, 0
    sock.sendto(('{"servo_jp": {"Goal": [%f, 0.0, 0.12, 0.0, 0.0, 0.0] } }' % counter).encode(), addr)
    # example of void command, incompatible with command above so uncomment only one
    # sock.sendto('{"hold": "" }'.encode(), addr)
