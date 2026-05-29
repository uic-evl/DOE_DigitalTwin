import cv2
import depthai as dai
import numpy as np
from collections import deque
import argparse
from depthai_utils import *
from aruco_utils import *
from kalman_utils import * 
from zmq_utils import * 

parser = argparse.ArgumentParser()
parser.add_argument("port", help="Port of ArUco tag pose publisher")
args = parser.parse_args()

# init tag detector
aruco_detector = create_aruco_detector(cv2.aruco.DICT_6X6_250)

# use size in meters from physical tag
MARKER_SIZE_METERS = 0.076
tag_corner_points = get_tag_corners(MARKER_SIZE_METERS)

# ArUco IDs for specific tags
ORIGIN_TAG_ID = 0
EE_TAG_ID = 24

### TBD ###
# Make relevant arrays global scope
t_cam_in_world = [0,0,0]
R_cam_in_world = [0,0,0]
###########

# Get intrinsics for center RGB camera
rgb_cam_matrix, rgb_dist_coeffs = set_intrinsics(get_intrinsics(dai.CameraBoardSocket.CAM_A))

pipeline, queues = create_pipeline()

# init kalman filters
kalman_filters = {}

# Diagnostics history
diag_history = {}
diag_length = 30  # sliding window

# init zmq socket
context, socket = set_zmq_socket(args.port)

# Running loop
if(pipeline != -1):
    pipeline.start()

    while pipeline.isRunning():

        # Get RGB output queue in openCV format
        videoIn = queues[1].get()
        frame = videoIn.getRGBFrame().getCvFrame()
    
        # detect tags
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)

        # If tags are detected...
        if ids is not None:
            ''' worry about this later
            # find index of origin and endpoint 
            origin_idx = np.where(ids == ORIGIN_TAG_ID)[0]
            endpoint_idx = np.where(ids == EE_TAG_ID)[0]
            '''

            # For each detected tag...
            for i, marker_id in enumerate(ids.flatten()):
                # PnP 3D point calc
                img_pts = corners[i].reshape(4, 2).astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    tag_corner_points,
                    img_pts,
                    rgb_cam_matrix,
                    rgb_dist_coeffs
                )

                if success:
                    # Draw 3D axis
                    draw_detection(frame, rgb_cam_matrix, rgb_dist_coeffs, rvec, tvec, MARKER_SIZE_METERS, marker_id, img_pts)
        
                    marker_id = int(marker_id)
                    R_xform = rvec
                    t_xform = tvec

                    # get world pos using filtered points
                    if(marker_id == ORIGIN_TAG_ID):
                        t_cam_in_world, R_cam_in_world = find_world_origin(rvec, tvec)

                        # Redundant for error checking of camera->world transform on ee tag
                        t_xform = t_cam_in_world
                        R_xform = R_cam_in_world

                    elif(marker_id == EE_TAG_ID):
                        t_ee_in_world, R_ee_in_world = find_tag_in_world(rvec, tvec, R_cam_in_world, t_cam_in_world)

                        t_xform = t_ee_in_world
                        R_xform = R_ee_in_world

                    # send msg 
                    zmq_send_msg(context, socket, marker_id, t_xform, cv_to_OVIS(R_xform))



        # visualize camera output queue
        cv2.imshow("RGBD Tagged", frame)

        if cv2.waitKey(1) == ord("q"):
            zmq_shutdown(context, socket)
            break
        
        
    cv2.destroyAllWindows()
else:
    print("Failed to Create depthai Pipeline")