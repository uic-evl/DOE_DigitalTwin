import cv2
import depthai as dai
import numpy as np
import time
import zmq
import argparse

def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to quaternion [w, x, y, z]."""
    trace = R[0,0] + R[1,1] + R[2,2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s

    return w, x, y, z

parser = argparse.ArgumentParser()
parser.add_argument("port", help="Port of ArUco tag pose publisher")
args = parser.parse_args()

# Luxonis Device Init
TARGET_MXID = "1944301091B1EF1200"   # Camera on tower
#TARGET_MXID = "14442C10A16B03D700"  # Camera on table

# ZMQ Socket Init
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(f"tcp://*:{args.port}")

cv_to_isaac = np.array([
    [ 0,  0,  1],   # Isaac X  =  OpenCV Z
    [-1,  0,  0],   # Isaac Y  = -OpenCV X
    [ 0, -1,  0]    # Isaac Z  = -OpenCV Y
], dtype=np.float64)

# ArUco Init
ARUCO_DICT = cv2.aruco.DICT_6X6_250
MARKER_SIZE_METERS = 0.076  # measured with ruler

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# 3D marker corner coordinates
obj_points = np.array([
    [-MARKER_SIZE_METERS/2,  MARKER_SIZE_METERS/2, 0],
    [ MARKER_SIZE_METERS/2,  MARKER_SIZE_METERS/2, 0],
    [ MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
    [-MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
], dtype=np.float32)

# From get_intrinsics.py
# These are from camera's board, MAY be wrong
camera_matrix = np.array([
    [454.0347595214844, 0.0, 316.25091552734375],
    [0.0, 454.0347595214844, 203.69068908691406],
    [0.0,     0.0,   1.0]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# Make relevant arrays global scope
t_cam_in_world = []
t_endpoint_in_world = []

# From Luxonis Chat
with dai.Pipeline() as p:
    # Create nodes
    left   = p.create(dai.node.Camera)
    right  = p.create(dai.node.Camera)
    color  = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd   = p.create(dai.node.RGBD).build()  # autocreate color/depth if desired

    # Build cameras
    color.build()
    left.build(dai.CameraBoardSocket.CAM_B)
    right.build(dai.CameraBoardSocket.CAM_C)

    # Stereo config
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.initialConfig.postProcessing.thresholdFilter.maxRange = 10000
    rgbd.setDepthUnits(dai.StereoDepthConfig.AlgorithmControl.DepthUnit.METER)

    # Link mono to stereo
    left.requestOutput((640, 400)).link(stereo.left)
    right.requestOutput((640, 400)).link(stereo.right)

    # RGB + depth → RGBD
    platform = p.getDefaultDevice().getPlatform() # This should automatically select an available device
    if platform == dai.Platform.RVC4:
        out = color.requestOutput((640, 400), dai.ImgFrame.Type.RGB888i, enableUndistortion=True)
        align = p.create(dai.node.ImageAlign)
        stereo.depth.link(align.input)
        out.link(align.inputAlignTo)
        align.outputAligned.link(rgbd.inDepth)
    else:
        out = color.requestOutput(
            (640, 400), dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 30, True
        )
        stereo.depth.link(rgbd.inDepth)
        out.link(stereo.inputAlignTo)

    out.link(rgbd.inColor)

    # Host queues for point cloud + RGBD
    qPcl  = rgbd.pcl.createOutputQueue()
    qRgbd = rgbd.rgbd.createOutputQueue()

    p.start()
    
# End From Luxonis Chat
    
    # This is the running loop
    while p.isRunning():
        videoIn = qRgbd.get()
        frame = videoIn.getRGBFrame().getCvFrame()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)

        # If tags are detected...
        if ids is not None:
            # Visualize tags
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # find index of origin and endpoint 
            origin_idx = np.where(ids == 0)[0]
            endpoint_idx = np.where(ids == 24)[0]

            # For now, the origin must be visible for any tracking to happen
            # In the future, I'll store the origin. 
            if origin_idx.size > 0 and endpoint_idx.size > 0:
                # Find tag_in_cam, then cam_in_world
                img_pts = corners[origin_idx[0]].reshape(4, 2).astype(np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_pts,
                    camera_matrix,
                    dist_coeffs
                )

                if success:
                    # Convert from camera to world coords
                    R_tag_in_cam, _ = cv2.Rodrigues(rvec)      
                    t_tag_in_cam = tvec.flatten()      

                    R_cam_in_world = R_tag_in_cam.T            
                    t_cam_in_world = -R_cam_in_world @ t_tag_in_cam  

                    R_isaac = cv_to_isaac @ R_cam_in_world @ cv_to_isaac.T
                    t_isaac = cv_to_isaac @ t_cam_in_world

                    # Convert rotation matrix to quaternion
                    qw, qx, qy, qz = rotation_matrix_to_quaternion(R_isaac)             
                    x, y, z = t_cam_in_world 

                    # Find endpoint_in_world
                    img_pts = corners[endpoint_idx[0]].reshape(4, 2).astype(np.float32)

                    success, rvec, tvec = cv2.solvePnP(
                        obj_points,
                        img_pts,
                        camera_matrix,
                        dist_coeffs
                    )

                    if success:
                        t_endpoint_in_cam = tvec.flatten()
                        t_endpoint_in_world = (R_cam_in_world @ t_endpoint_in_cam) + t_cam_in_world 

                        R_endpoint_in_cam, _ = cv2.Rodrigues(rvec) 
                        R_endpoint_in_world = R_cam_in_world @ R_endpoint_in_cam

                        Re_isaac = cv_to_isaac @ R_endpoint_in_world @ cv_to_isaac.T
                        #t_isaac = cv_to_isaac @ t_endpoint_in_world
                        
                        #tqw, tqx, tqy, tqz = rotation_matrix_to_quaternion(R_endpoint_in_world) 
                        tqw, tqx, tqy, tqz = rotation_matrix_to_quaternion(Re_isaac) 
                        tx, ty, tz = t_endpoint_in_world

                        try:
                            message = str(x) + ',' + str(y) + ',' + str(z) + ',' + str(qx) + ',' + str(qy) + ',' + str(qz) + ',' + str(qw)
                            message2 = str(tx) + ',' + str(ty) + ',' + str(tz) + ',' + str(tqx) + ',' + str(tqy) + ',' + str(tqz) + ',' + str(tqw)
                            print(message)
                            message_full = message + ";" + message2
                            socket.send_string(message_full)
                            time.sleep(0.1) #Probably not needed, this is mostly for testing
                        except KeyboardInterrupt:
                            socket.close()
                            context.term()
                            break
                        except zmq.ZMQError as e:
                            print("Error: " + str(e))
                            break
    
            # Stream the coordinates to OVIS

            #print("origin to camera is " + str(t_cam_in_world))
            print("origin to endpoint tag is" + str(t_endpoint_in_world))
            

        cv2.imshow("RGBD Tagged", frame)

        if cv2.waitKey(1) == ord("q"):
            socket.close()
            context.term()
            break


