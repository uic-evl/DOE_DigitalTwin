#
# This file is to link messages from Omniverse to the robot
# Author: Athena Angara

import time 
import zmq  
from pymycobot.myarm import MyArm  
import math  

# Initialize the robot arm with the serial port
# This sets up communication with the robot arm
mc = MyArm('/dev/ttyAMA0')

# Initial angles for the robot in degrees
# The robot will move to these positions at the start
init_angles = [
    [90, 0, 0, 0, 0, 0, 0],  # zero point
    [-45, 79.9, -20.4, -90.9, -10, 44, 76],  # first init point
]

# Define speed levels for the robot's movements
low_speed = 10
medium_speed = 50
high_speed = 100

# Define a delay time
timet = int(3)

# Set up ZMQ context and socket for receiving messages
context = zmq.Context()
socket = context.socket(zmq.SUB)

# Bind the socket to a TCP port to listen for messages
socket.bind('tcp://*:5560')

# Subscribe to all incoming messages
socket.subscribe('')

# Function to convert angles from radians to degrees
# The robot expects degrees, but we'll receive radians
def radians_to_degrees(radians):
    return [math.degrees(angle) for angle in radians]

def main():
    while True:
        try:
            # Wait for a message from the socket
            message = socket.recv_multipart()

            # Print the received message to verify that it is being received correctly
            print("Received message:", message)

            # Extract and convert the relevant parts of the message (joint angles in radians)
            joint_angles_radians = [float(angle) for angle in message[1:]]  # Ignore the first element

            # Convert the joint angles from radians to degrees
            joint_angles_degrees = radians_to_degrees(joint_angles_radians)

            # Send the converted angles to the robot at low speed
            mc.send_angles(joint_angles_degrees, low_speed)

            # Wait for a while before the next operation to avoid rapid commands
            #time.sleep(10 + timet)

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
