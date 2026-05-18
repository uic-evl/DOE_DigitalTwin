#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

# -------------------------------
# ArUco configuration
# -------------------------------
ARUCO_DICT = cv2.aruco.DICT_6X6_250
MARKER_SIZE_METERS = 0.077  # <-- CHANGE to your tag size

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# -------------------------------
# Approximate intrinsics (RGB camera)
# Replace with calibrated values for accuracy
# -------------------------------
camera_matrix = np.array([
    [900.0,   0.0, 960.0],
    [0.0,   900.0, 540.0],
    [0.0,     0.0,   1.0]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# 3D marker corner coordinates
obj_points = np.array([
    [-MARKER_SIZE_METERS/2,  MARKER_SIZE_METERS/2, 0],
    [ MARKER_SIZE_METERS/2,  MARKER_SIZE_METERS/2, 0],
    [ MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
    [-MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
], dtype=np.float32)

# -------------------------------
# Create device + pipeline
# -------------------------------
device = dai.Device()

with dai.Pipeline(device) as pipeline:
    outputQueues = {}

    sockets = device.getConnectedCameras()
    for socket in sockets:
        cam = pipeline.create(dai.node.Camera).build(socket)
        outputQueues[str(socket)] = cam.requestFullResolutionOutput().createOutputQueue()

    pipeline.start()

    print("press 'q' to quit")

    while pipeline.isRunning():
        for name, queue in outputQueues.items():
            videoIn = queue.get()
            frame = videoIn.getCvFrame()

            # Only run pose estimation on RGB / CAM_A
            if name == str(dai.CameraBoardSocket.CAM_A):
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                corners, ids, _ = aruco_detector.detectMarkers(gray)

                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                    for i, marker_id in enumerate(ids.flatten()):
                        img_pts = corners[i].reshape(4, 2).astype(np.float32)

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

                            print(
                                f"Tag {marker_id} | "
                                f"x={x:.3f} y={y:.3f} z={z:.3f} m"
                            )

            cv2.imshow(name, frame)

        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()