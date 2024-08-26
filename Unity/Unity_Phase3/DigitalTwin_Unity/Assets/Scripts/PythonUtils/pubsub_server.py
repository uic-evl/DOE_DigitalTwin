# ZMQ Sample Publisher

import time
import random
import zmq

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:12344")
time_interval = 1.0

limit = 2
iter = 0

while iter < limit:#True:
    iter += 1
    try:
        # send message every time_interval seconds
        message = str(random.uniform(-1.0,1.0)) + " " + str(random.uniform(-1.0,1.0))
        print(message)
        socket.send_string(message)
        time.sleep(time_interval)
    except KeyboardInterrupt:
        socket.close()
        context.term()
        break
    except zmq.ZMQError as e:
        print("Error: " + str(e))
        break

socket.close()
context.term()