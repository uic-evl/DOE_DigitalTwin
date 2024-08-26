# This is a debugging script. You can comment out the "helloN" statements as needed. 
# This will let you make sure teh ZMQ connection is working. 
import zmq

# Set up context and socket
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("hello")
# Bind the socket to a port
socket.bind('tcp://*:5560')
print("hello2")
# Subscribe to all incoming messages
socket.subscribe('')
print("hello3")
while True:
    try:
        print("hello4")
        # Wait for a message
        message = socket.recv_multipart()
        print("hello5")
        #message = socket.recv()
        print("hello6")
        # Process the message here
        print(message)#from {address}')
        print("hello7")
    except KeyboardInterrupt:
        break

# Clean up the socket and context
print("hello8")
socket.close()
print("hello9")
context.term()
print("hello10")
zmq_ctx_destroy()
