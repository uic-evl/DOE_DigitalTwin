#
# This file is to link messages from Omniverse to the robot
# Author: Athena Angara

import time 
import zmq  
from pymycobot.mycobot import MyCobot  
import math  
import serial

# Initialize the robot arm with the serial port
# This sets up communication with the robot arm
mc = MyCobot('/dev/ttyTHS1',1000000)

# Initial angles for the robot in degrees
# The robot will move to these positions at the start
init_angles = [
    [90, 0, 0, 0, 0, 0],  # zero point
    [-45, 79.9, -20.4, -90.9, -10, 44],  # first init point
]

# Define speed levels for the robot's movements
low_speed = 10
medium_speed = 50
high_speed = 100

# Define a delay time
timet = int(3)

# Set up ZMQ context and socket for receiving messages
context = zmq.Context()
socket = context.socket(zmq.PUB)

socket.connect('tcp://*:5560')

# Function to convert angles from radians to degrees
# The robot expects degrees, but we'll receive radians
def radians_to_degrees(radians):
    return [math.degrees(angle) for angle in radians]

def degrees_to_radians(degrees):
    return [math.radians(angle) for angle in degrees]

def main():
    while True:
        try:
            # Extract and convert the relevant parts of the message (joint angles in radians)
            joint_angles_degrees = mc.get_angles()

            # Convert the joint angles from radians to degrees
            joint_angles_radians = degrees_to_radians(joint_angles_degrees)
            
            joint_angles_radians = [str(j).encode() for j in joint_angles_radians]
            
            print("sending...")
            socket.send_multipart([b'Get', *joint_angles_radians])
            print(joint_angles_radians)

            # Wait for a while before the next operation to avoid rapid commands
            #time.sleep(0.0001 + timet)

        except KeyboardInterrupt:
            # Handle the script being stopped by the user
            break
        except ValueError as e:
            # Handle any conversion errors
            print(f"Error converting message: {e}")
            continue

    # Clean up the socket and context
    socket.close()
    context.term()


if __name__ == '__main__':
    main()
