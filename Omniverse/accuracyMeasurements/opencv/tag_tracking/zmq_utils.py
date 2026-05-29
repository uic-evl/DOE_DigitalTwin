import zmq

def set_zmq_socket(port):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{port}")

    return context, socket

def zmq_shutdown(context, socket):
    socket.close()
    context.term()

def zmq_send_msg(context, socket, marker_id, t_data, R_data):
    try:
        topic = f"tag_{marker_id}"
        msg_data = f"{t_data[0]:.4f},{t_data[1]:.4f},{t_data[2]:.4f},{R_data[0]:.4f},{R_data[1]:.4f},{R_data[2]:.4f},{R_data[3]:.4f}"
        print(f"{topic} {msg_data}")
        socket.send_string(f"{topic} {msg_data}")
    
    except KeyboardInterrupt:
        socket.close()
        context.term()
    
    except zmq.ZMQError as e:
        print("Error: " + str(e))