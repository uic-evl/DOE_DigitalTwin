import cv2
import zmq
import base64
import time
import math
from pymycobot.mycobot import MyCobot
# Initialize the camera
cap = cv2.VideoCapture(0)

# Initialize ZeroMQ context and sockets
context = zmq.Context()
camera_socket = context.socket(zmq.PUB)
camera_socket.bind("tcp://*:5555")

command_socket = context.socket(zmq.SUB)
command_socket.bind('tcp://*:5560')
command_socket.subscribe('')

# Initialize the MyCobot
mc = MyCobot('/dev/ttyTHS1', 1000000)

# Function to convert angles from radians to degrees
def radians_to_degrees(radians):
    return [math.degrees(angle) for angle in radians]

def main():
    while True:
        try:
            # Camera capture and send
            ret, frame = cap.read()
            if not ret:
                break
            
            # Encode the frame in JPEG format
            _, buffer = cv2.imencode('.jpg', frame)
            encoded_frame = base64.b64encode(buffer)
            
            # Send the encoded frame
            camera_socket.send(encoded_frame)
            
            # Display the frame locally (optional)
            cv2.imshow('Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Robot command to receive and execute
            try:
                # Wait for a message from the socket
                message = command_socket.recv_multipart(zmq.NOBLOCK)
                
                # Print the received message to verify that it is being received correctly
                print("Received message:", message)

                # Extract and convert the relevant parts of the message (joint angles in radians)
                joint_angles_radians = [float(angle) for angle in message[1:]]  # Ignore the first element

                # Convert the joint angles from radians to degrees
                joint_angles_degrees = radians_to_degrees(joint_angles_radians)

                # Send the converted angles to the robot at medium speed
                mc.send_angles(joint_angles_degrees, 50)

            except zmq.Again:
                # No message received, continue looping
                pass

        except KeyboardInterrupt:
            # Handle the script being stopped by the user
            break
        except ValueError as e:
            # Handle any conversion errors
            print(f"Error converting message: {e}")
            continue

    # Clean up the sockets and context
    cap.release()
    cv2.destroyAllWindows()
    camera_socket.close()
    command_socket.close()
    context.term()

if __name__ == '__main__':
    main()
