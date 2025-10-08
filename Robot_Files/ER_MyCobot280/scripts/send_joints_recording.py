#
# Send current robot joints to Omniverse
#

import time 
import datetime
import zmq    
from pymycobot import MyCobot
import math  
import sys
import serial

# Initialize the robot arm with the serial port
# This sets up communication with the robot arm
mc = MyCobot('/dev/ttyTHS1',1000000)

# Set up ZMQ context and socket for receiving messages
context = zmq.Context()
socket = context.socket(zmq.PUB)

# Bind the socket to a TCP port to listen for messages
# Address should match the machine with an Isaac Sim Publisher
socket.bind('tcp://*:12348') # UPDATE IP WITH IP OF SIMULATION MACHINE

# Function to convert angles from radians to degrees
# The robot expects degrees, but we'll receive radians
def radians_to_degrees(radians):
    return [math.degrees(angle) for angle in radians]

def degrees_to_radians(degrees):
    return [math.radians(angle) for angle in degrees]


def main():
    
    print("Running")
    # Open file for logging
    filestream = open("/home/er/py_scripts/Recordings/real_arm_applied.txt", "w", encoding="utf-8")

    while True:
        try:
            #print("loop")

            message = mc.get_angles()

            if message is None:
                continue
            else:
                message = degrees_to_radians(message)
                socket.send_string(str(message))
                printData = str(datetime.datetime.now()) + ";" +  str(message) + "\n"
                filestream.writelines(printData)
            
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
        except serial.serialutil.SerialException:
            continue

    # Clean up the socket and context
    socket.close()
    context.term()
    

    # Close the file stream
    filestream.close()

if __name__ == '__main__':
    main()
