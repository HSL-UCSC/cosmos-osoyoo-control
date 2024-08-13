# to be run on a laptop w/ the Radiomaster Zorro connected
import json

import awsdeepracer_control as dr
import hid

h = None
ip = "128.114.59.181"

try:
    # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client = dr.Client(password="WThn8DOx", ip=ip)
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

    print("Set manual mode:", client.set_manual_mode())
    print("Start car:", client.start_car())

    print("Opening the device")

    h = hid.device()
    h.open(4617, 20308)

    print("Manufacturer: %s" % h.get_manufacturer_string())
    print("Product: %s" % h.get_product_string())
    print("Serial No: %s" % h.get_serial_number_string())

    # enable non-blocking mode
    h.set_nonblocking(1)

    def send(throttle, angle):
        print(
            f"Sending {throttle}, {angle}, {client.move(angle, -1.0 * throttle, 1.0)}"
        )

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
            send(0, 0)
        elif reverse:
            send(-l_y, (r_x - 0.5) * 2)
        else:
            send(l_y, (r_x - 0.5) * 2)

except IOError as ex:
    print(ex)

except KeyboardInterrupt:
    if h:
        print("Closing the device")
        h.close()

print("Done")
