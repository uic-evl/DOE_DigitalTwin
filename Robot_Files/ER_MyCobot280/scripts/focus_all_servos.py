
from pymycobot import MyCobot
import time

mc = MyCobot('/dev/ttyTHS1',1000000)

mc.focus_servo(1)
time.sleep(0.5)
mc.focus_servo(2)
time.sleep(0.5)
mc.focus_servo(3)
time.sleep(0.5)
mc.focus_servo(4)
time.sleep(0.5)
mc.focus_servo(5)
time.sleep(0.5)
mc.focus_servo(6)
time.sleep(0.5)
