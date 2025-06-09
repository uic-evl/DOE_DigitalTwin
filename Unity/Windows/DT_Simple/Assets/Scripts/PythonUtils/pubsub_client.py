# ZMQ Sample Subscriber

import time
import random
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect('tcp://140.221.17.45:12325')
socket.subscribe('')
time_interval = 0.01

while True:
    try:
        # send message every time_interval seconds
        message = socket.recv_string(flags=zmq.NOBLOCK)
        print("Got: ", message)
        time.sleep(time_interval)
    except KeyboardInterrupt:
        # Handle the script being stopped by the user
        # Clean up the socket and context
        socket.close()
        context.term()
        break
    except zmq.ZMQError as e:

        if e.errno == zmq.EAGAIN:
            pass # no message was ready (yet!)
        else:
            print("Error: " + str(e))
            break

socket.close()
context.term()
