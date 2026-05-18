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

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                img_pts = corners[i].reshape(4, 2).astype(np.float32)

                # This gets the XYZ of the four corners from the point cloud
                '''
                for corner in img_pts:
                    # Map 2D region to 3D points
                    pclData = qPcl.get()

                    if pclData.isSparse() == False:
                        width = pclData.getWidth()
                        height = pclData.getHeight()

                        # Convert pixel to linear index
                        i = (corner[1] * width) + corner[0]; 

                        points = pclData.getPoints()  
                        point = points[int(i)]
                        print(point)
                '''

                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_pts,
                    camera_matrix,
                    dist_coeffs
                )

                if success:
                    # Tag Detection Visualization
                    # Draw 3D axis
                    cv2.drawFrameAxes(
                        frame,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        MARKER_SIZE_METERS * 0.5
                    )

                    x, y, z = tvec.flatten()
                    rx, ry, rz = rvec.flatten()
                    label = f"ID {marker_id}  X:{x:.2f} Y:{y:.2f} Z:{z:.2f} m"

                    cv2.putText(
                        frame,
                        label,
                        tuple(img_pts[0].astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                    # End visualization

                    if(marker_id == 0):
                        print("I see the origin!")
                    elif(marker_id == 24):
                        print("I see the ee!")
                    else:
                        print("Invalid marker ID")

                    # Convert from camera to world coords
                    R_tag_in_cam, _ = cv2.Rodrigues(rvec)      
                    t_tag_in_cam = tvec.flatten()               

                    R_cam_in_world = R_tag_in_cam.T            
                    t_cam_in_world = -R_cam_in_world @ t_tag_in_cam  

                    R_isaac = cv_to_isaac @ R_cam_in_world @ cv_to_isaac.T
                    t_isaac = cv_to_isaac @ t_cam_in_world

                    # Convert rotation matrix to quaternion
                    qw, qx, qy, qz = rotation_matrix_to_quaternion(R_cam_in_world)             
                    x, y, z = t_cam_in_world 
                    
                    # Stream the coordinates from tvec and rvec to OVIS
                    try:
                        message = str(x) + ',' + str(y) + ',' + str(z) + ',' + str(qx) + ',' + str(qy) + ',' + str(qz) + ',' + str(qw)
                        print(message)
                        socket.send_string(message)
                        time.sleep(0.1) #Probably not needed, this is mostly for testing
                    except KeyboardInterrupt:
                        socket.close()
                        context.term()
                        break
                    except zmq.ZMQError as e:
                        print("Error: " + str(e))
                        break

        cv2.imshow("RGBD Tagged", frame)

        if cv2.waitKey(1) == ord("q"):
            socket.close()
            context.term()
            break


