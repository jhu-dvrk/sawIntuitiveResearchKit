import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 48051 # look for port in share/socket-streamer/streamerXXX.json

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind((UDP_IP, UDP_PORT))
counter = 0.0;
increment = 0.001

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print('received message: %s' % data)
    # change direction to stay in range [-0.5, 0.5]
    if counter < -0.5 or counter > 0.5:
        increment *= -1.0
    counter += increment
    sock.sendto(('{"servo_jp": {"Goal": [%f, 0.0, 0.12, 0.0, 0.0, 0.0] } }' % counter).encode(), addr)
