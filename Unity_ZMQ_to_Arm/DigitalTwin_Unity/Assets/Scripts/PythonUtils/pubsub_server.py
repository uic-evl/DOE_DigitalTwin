import time
import random
import zmq

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:12346")
time_interval = 1.0

while True:
    try:
        # send message every time_interval seconds
        message = str(random.uniform(-1.0,1.0)) + " " + str(random.uniform(-1.0,1.0))
        print(message)
        socket.send_string(message)
        time.sleep(time_interval)
    except KeyboardInterrupt:
        # Handle the script being stopped by the user
        # Clean up the socket and context
        socket.close()
        context.term()
        break

