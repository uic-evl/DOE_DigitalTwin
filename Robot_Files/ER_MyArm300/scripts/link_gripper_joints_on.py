#
# This file is to recieve messages from Omniverse to the robot
# Author: Athena Angara
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
socket.connect('tcp://10.0.0.147:12346')

# Subscribe to all incoming messages
socket.subscribe('')

# Function to convert angles from radians to degrees
# The robot expects degrees, but we'll receive radians
def radians_to_degrees(radians):
    return [math.degrees(angle) for angle in radians]

def main():

    #json_angles=[]
    sendAngles = True
    waitEndTime = datetime.datetime.now()
    currentTime = datetime.datetime.now()
    timeDelay = 0.1

    while True:
        try:
            # Wait for a message from the socket
            message = socket.recv_string(flags=zmq.NOBLOCK)
            #message = socket.recv_string()

            # Print the received message to verify that it is being received correctly
            #print("Received message:", message)
            
            # Clean String
            message = message[1:-1]
            message = message.replace("'", "")
            message = message.replace("b", "")
            message_arr = message.split(",")

            message_arr = message_arr[:7]

            # Extract and convert the relevant parts of the message (joint angles in radians)
            #joint_angles_radians = [float(angle) for angle in message[1:]]  # Ignore the first element
            #print(message_arr)
            joint_angles_radians = [float(angle) for angle in message_arr]
            #print(joint_angles_radians)
            
            # Convert the joint angles from radians to degrees
            joint_angles_degrees = radians_to_degrees(joint_angles_radians)
            #json_angle.append(joint_angles_degrees)
            #for i in range (0,6):
                #if(i == 0):
                    #joint_angles_degrees[i] = joint_angles_degrees[i]# * -1.0
                    #print(str(joint_angles_degrees[i]) + " negated is " + str(joint_angles_degrees[i] * -1.0))
             
            #print(joint_angles_degrees)
            #json_angle.append(joint_angles_degrees)
            # Send the converted angles to the robot at low speed
            if(sendAngles):
                waitEndTime = datetime.datetime.now() + datetime.timedelta(seconds=timeDelay)
                #print("Set angles")
                #print(joint_angles_degrees)
                
                #my_angles = mc.get_angles()

                #print(str(datetime.datetime.now()) + "," + str(my_angles))

                mc.send_angles(joint_angles_degrees, high_speed)
                sendAngles = False

            if(datetime.datetime.now() >= waitEndTime):
                sendAngles = True


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
