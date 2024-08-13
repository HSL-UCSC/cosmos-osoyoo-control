# to be run on the deepracer
import json
import socket
import struct

import awsdeepracer_control

ip = "0.0.0.0"
port = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ip, port))

client = awsdeepracer_control.Client(password="WThn8DOx", ip="127.0.0.1")
info = {
    "USB connection": client.get_is_usb_connected(),
    "Battery level": client.get_battery_status(),
    "Angle settings": client.get_calibration_angle(),
    "Throttle settings": client.get_calibration_throttle(),
    "Models": client.get_models(),
    "Network details": client.get_network_details(),
}
for key, value in info.items():
    print(f"{key}: {json.dumps(value, indent=4)}")
# client.show_vehicle_info()

print(client.set_manual_mode())
print(client.start_car())

try:
    while True:
        data, addr = sock.recvfrom(1024)
        throttle, angle = struct.unpack("2d", data)
        print(throttle, angle, client.move(angle, -1.0 * throttle, 1.0))
except:
    client.stop_car()