#!/usr/bin/python
# -*- coding:utf-8 -*-
# @File    : all_run_test.py
# @Author  : Wang Weijian
# @Time    :  2023/07/11 15:38:45
# @function: the script is used to do something
# @version : V1

import time
from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0')

init_angles = [
    [90, 0, 0, 0, 0, 0, 0],  # zero point
    #[-45, 79.9, -20.4, -90.9, -10, 44, 76],  # first init point
]

low_speed = 10
medium_speed = 50
high_speed = 100 
timet = int(3) #delay


def main():

    mc.send_angles(init_angles[0], low_speed)
    time.sleep(10+timet)

   
if __name__ == '__main__':
    main()
