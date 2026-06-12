
from pymycobot import MyCobot
import time

mc = MyCobot('/dev/ttyTHS1',1000000)

print("Releasing servos")

mc.release_all_servos()

while(True):
    try:
        print(mc.get_angles())
    except KeyboardInterrupt:
        break


print("Refocusing servos...")
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

print("Done refocus, robot locked.")

#home = [0,0,0,0,0,0]

#mc.send_angles(home, 50)

print("All done")
