import time
import random
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect('tcp://localhost:12345')
socket.subscribe('')
time_interval = 0.01

while True:
    try:
        # send message every time_interval seconds
        message = socket.recv_string()
        print("Got: ", message)
        time.sleep(time_interval)
    except KeyboardInterrupt:
        # Handle the script being stopped by the user
        # Clean up the socket and context
        socket.close()
        context.term()
        break
    except ValueError as e:
        # Handle any conversion errors
        print("Message Recv Failed")
        continue

