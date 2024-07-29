import socket
import struct
from pynput import keyboard
from tkinter import *


steering_angle = 0
velocity = 0
ip = '192.168.0.208'
port = 10002


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


root = Tk()
root.geometry("300x200")
root.title("Control Status")
key_status = StringVar()
key_status.set("No key is pressed")
label = Label(root, textvariable=key_status, font=("Helvetica", 16))
label.pack()

def send_command(steering_angle, velocity):
   
    steering_angle = int(steering_angle) + 40
    velocity = int(velocity)
    print(steering_angle, velocity)
    data = struct.pack('BB', steering_angle, velocity)

   
    sock.sendto(data, (ip, port))


def on_press(key):
    global velocity
    global steering_angle

    try:
        new_velocity = velocity
        new_steering_angle = steering_angle
        if key.char == 'w':
            new_velocity = 100  
            key_status.set("'W' key is pressed")
        elif key.char == 's':
            # new_velocity = -100  
            new_velocity = 255  
            key_status.set("'S' key is pressed")
        elif key.char == 'a':
            new_steering_angle = 30  
            key_status.set("'A' key is pressed")
        elif key.char == 'd':
            new_steering_angle = -30  
            key_status.set("'D' key is pressed")

        if new_velocity != velocity or new_steering_angle != steering_angle:
            velocity = new_velocity
            steering_angle = new_steering_angle
            send_command(steering_angle, velocity)  
    except AttributeError:
        pass


def on_release(key):
    global velocity
    global steering_angle

    try:
        new_velocity = velocity
        new_steering_angle = steering_angle
        if key.char == 'w' or key.char == 's':
            new_velocity = 0  
            key_status.set("No key is pressed")
        elif key.char == 'a' or key.char == 'd':
            new_steering_angle = 0  
            key_status.set("No key is pressed")

        if new_velocity != velocity or new_steering_angle != steering_angle:
            velocity = new_velocity
            steering_angle = new_steering_angle
            send_command(steering_angle, velocity)  
    except AttributeError:
        pass


listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()


root.mainloop()


sock.close()
