import cv2
import depthai as dai
import numpy as np
import time

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
    platform = p.getDefaultDevice().getPlatform()
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
    pclData  = qPcl.get()   # dai.PointCloudData
    rgbdData = qRgbd.get()  # dai.RGBDData

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
                    
                    '''
                    print(
                        f"Tag {marker_id} | "
                        f"x={x:.3f} y={y:.3f} z={z:.3f} m"
                    )
                    '''

                    # Stream the coordinates from tvec and rvec to OVIS

        cv2.imshow("RGBD Tagged", frame)

        if cv2.waitKey(1) == ord("q"):
            break


