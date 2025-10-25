
# This file is to link messages from Omniverse to the robot with gripper
# Author: Hal Brynteson 
#

import time 
import datetime
import zmq  
from pymycobot.myarm import MyArm  
import math  
import sys

# Initialize the robot arm with the serial port
# This sets up communication with the robot arm
mc = MyArm('/dev/ttyAMA0',115200)

# Define speed levels for the robot's movements
low_speed = 10
medium_speed = 50
high_speed = 100

# Set up ZMQ context and socket for receiving messages
context = zmq.Context()
socket = context.socket(zmq.SUB)

# Bind the socket to a TCP port to listen for messages
# Address should match the machine with an Isaac Sim Publisher
# Links with "send_gripper.py"
socket.connect('tcp://10.0.0.147:12348')

# Subscribe to all incoming messages
socket.subscribe('')


#function is to set gripper state from message
def gripper_state(switch_state):
    state = float(switch_state)
    if state == 0:
        pass
    elif state == 1:
        #print('open')
        mc.set_gripper_state(0,medium_speed,1)# the 1 is for the adaptive gripper, ensure correct gripper 
    elif state == 2:
        #print('close')
        mc.set_gripper_state(1,medium_speed,1)
    elif state == 3:
        #print('Release')
        mc.set_gripper_state(0,medium_speed,1)#dont ask idk why 254
    else:
        print('gripper_state function did not recieve switch_state as int in inclusive range [0 - 3]')
        pass


def main():

    while True:
        try:
            # Wait for a message from the socket
            #message = socket.recv_multipart(flags=zmq.NOBLOCK)
            #print("Running")

            message = socket.recv_string(flags=zmq.NOBLOCK)
            switch_state = message
            #print(switch_state)
        
            gripper_state(switch_state)

        except KeyboardInterrupt:
            # Handle the script being stopped by the user
            break
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass
            else:
                # Handle any conversion errors
                print(e)
                continue

    # Clean up the socket and context
    socket.close()
    context.term()


if __name__ == '__main__':
    main()
