# to be run on a laptop w/ the Radiomaster Zorro connected
import socket
import struct

import hid

h = None
ip = "128.114.59.181"
port = 8888

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Opening the device")

    h = hid.device()
    h.open(4617, 20308)

    print("Manufacturer: %s" % h.get_manufacturer_string())
    print("Product: %s" % h.get_product_string())
    print("Serial No: %s" % h.get_serial_number_string())

    # enable non-blocking mode
    h.set_nonblocking(1)

    last_input = [0, 0]
    throttle_threshold = 0.1
    angle_threshold = 0.01

    def send(inputs, throttle, angle):
        print(f"Sending {throttle}, {angle}")
        if (
            inputs
            and abs(throttle - inputs[0]) < throttle_threshold
            and abs(angle - inputs[1]) < angle_threshold
            and throttle > 0.05
        ):
            return

        sock.sendto(struct.pack("2d", throttle, angle), (ip, port))
        inputs = [throttle, angle]

    # read back the answer
    print("Read the data")
    while True:
        d = h.read(64)
        if not d:
            continue

        # print(d)
        r_x = d[3] | (d[4] << 8)
        r_y = d[5] | (d[6] << 8)

        l_y = d[7] | (d[8] << 8)
        l_x = d[9] | (d[10] << 8)

        r_x /= 1986

        r_y /= 2007

        l_x -= 45
        l_x /= 2047 - 45

        l_y /= 2008

        reverse = (d[11] | (d[12] << 8)) != 0

        arm = (d[13] | (d[14] << 8)) != 0

        if not arm:
            send(None, 0, 0)
        elif reverse:
            send(last_input, -l_y, (r_x - 0.5) * 2)
        else:
            send(last_input, l_y, (r_x - 0.5) * 2)

except IOError as ex:
    print(ex)

except KeyboardInterrupt:
    if h:
        print("Closing the device")
        h.close()

print("Done")
