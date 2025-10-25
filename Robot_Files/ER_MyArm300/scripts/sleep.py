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


def main():

    for i in range(1,8):
        mc.release_servo(i)
   
if __name__ == '__main__':
    main()
