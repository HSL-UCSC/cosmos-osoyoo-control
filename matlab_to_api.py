import socket
import struct

import awsdeepracer_control

ip = "0.0.0.0"
port = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ip, port))

client = awsdeepracer_control.Client(password="WThn8DOx", ip="127.0.0.1")
client.show_vehicle_info()

print(client.set_manual_mode())
print(client.start_car())

while True:
    data, addr = sock.recvfrom(1024)
    throttle, angle = struct.unpack("2d", data)
    print(client.move(throttle, angle, 1.0))
