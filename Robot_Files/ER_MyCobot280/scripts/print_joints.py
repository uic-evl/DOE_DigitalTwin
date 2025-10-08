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

print(mc.get_angles())

time.sleep(1)
