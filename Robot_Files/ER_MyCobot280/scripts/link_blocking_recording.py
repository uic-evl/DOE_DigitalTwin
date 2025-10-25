#
# This file is to recieve messages from Omniverse to the robot
# apply the command, wait until the command has executed, 
# record the resulting state, then continue to recv
#

import time 
import datetime
import zmq    
from pymycobot import MyCobot
import math  
import sys

# Initialize the robot arm with the serial port
# This sets up communication with the robot arm
mc = MyCobot('/dev/ttyTHS1',1000000)

# Initial angles for the robot in degrees
# The robot will move to these positions at the start
init_angles = [90, 0, 0, 0, 0, 0, 0]  # zero point

# Define speed levels for the robot's movements
low_speed = 10
medium_speed = 50
high_speed = 100

# Define a delay time
timet = int(1)

# Set up ZMQ context and socket for receiving messages
context = zmq.Context()
socket = context.socket(zmq.SUB)

# Bind the socket to a TCP port to listen for messages
# Address should match the machine with an Isaac Sim Publisher
socket.connect('tcp://192.168.1.239:12346') # UPDATE IP WITH IP OF SIMULATION MACHINE

# Subscribe to all incoming messages
socket.subscribe('')

# Function to convert angles from radians to degrees
# The robot expects degrees, but we'll receive radians
def radians_to_degrees(radians):
    return [math.degrees(angle) for angle in radians]

def main():
    
    print("Running")

    sendAngles = True
    waitEndTime = datetime.datetime.now()
    currentTime = datetime.datetime.now()
    timeDelay = 0.1

    # Open file for logging 
    filestream = open("/home/er/py_scripts/Recordings/real_arm_waiting.txt", "w", encoding="utf-8")

    # Save last applied cmd
    lastCmdID = 0

    while True:
        try:
            # Wait for a message from the socket
            message = socket.recv_string()

            # Print the received message to verify that it is being received correctly
            #print("Received message:", message)
            
            # Process message string
            message = message.replace("'", "")
            message = message.replace("b", "")
            message = message.replace("[", "")
            message = message.replace("]", "")
            message_arr = message.split(",")

            cmdID = int(message_arr[0])
            message_arr = message_arr[1:]

            #print("After elem drop: " + str(message_arr))

            # Extract and convert the relevant parts of the message (joint angles in radians)
            #joint_angles_radians = [float(angle) for angle in message[1:]]  # Ignore the first element
            #print(message_arr)
            joint_angles_radians = [float(angle) for angle in message_arr]
            #print(joint_angles_radians)
            
            # Convert the joint angles from radians to degrees
            joint_angles_degrees = radians_to_degrees(joint_angles_radians)
            
            #print(joint_angles_degrees)
            
            # Send the converted angles to the robot at medium speed ONLY if last cmd is done
            if mc.is_moving() == 0:
                # Record the result of the previous command
                #print("Recording")
                printData = str(lastCmdID) + ";" + str(datetime.datetime.now()) + ";" +  str(mc.get_angles()) + "\n"
                filestream.writelines(printData)
                print(printData)

                # Apply new command
                #print(joint_angles_degrees)
                mc.send_angles(joint_angles_degrees, medium_speed)
                lastCmdID = cmdID

            elif mc.is_moving() == 1:
                #print("Moving...")
                continue

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

    # Close the file stream
    filestream.close()


if __name__ == '__main__':
    main()
