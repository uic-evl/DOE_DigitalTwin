
import zmq
import time  # Ensure clients have time to connect

import depthai as dai
import numpy as np
import cv2
import sys
import struct
from pathlib import Path
import json
import open3d as o3d

context = zmq.Context()
sock = context.socket(zmq.PUB)
sock.bind("tcp://*:12341")  # Bind to port

# time.sleep(1)  # Wait for subscribers to connect

print("Server is running...")

# Get argument first
nnBlobPath = str((Path(__file__).parent / Path('../models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnBlobPath = sys.argv[1]

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]



# Create camera nodes
pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
pointcloud: dai.node.PointCloud = pipeline.create(dai.node.PointCloud)
sync = pipeline.create(dai.node.Sync)
xOut = pipeline.create(dai.node.XLinkOut)
xOut.input.setBlocking(False)

# Output nodes for data streaming
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutDepth.setStreamName("depth")


# Configu the MobileNet spatial detection network
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)


# Config GB camera
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setIspScale(2,3)

camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# deapth settings
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(True)
depth.setExtendedDisparity(False)
depth.setSubpixel(True)
depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)
depth.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

config = depth.initialConfig.get()
config.postProcessing.thresholdFilter.minRange = 200
config.postProcessing.thresholdFilter.maxRange = 1000
depth.initialConfig.set(config)

# Config the MobileNet spatial detection network
spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)


# Configure and link pipeline nodes
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.depth.link(pointcloud.inputDepth)
camRgb.isp.link(sync.inputs["rgb"])
pointcloud.outputPointCloud.link(sync.inputs["pcl"])
pointcloud.initialConfig.setSparse(False)
sync.out.link(xOut.input)
xOut.setStreamName("out")

inConfig = pipeline.create(dai.node.XLinkIn)
inConfig.setStreamName("config")
inConfig.out.link(pointcloud.inputConfig)

camRgb.preview.link(spatialDetectionNetwork.input)

spatialDetectionNetwork.passthrough.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)

depth.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)



with dai.Device(pipeline) as device:

    isRunning = True
    def key_callback(vis, action, mods):
        global isRunning
        if action == 0:
            isRunning = False

    q = device.getOutputQueue(name="out", maxSize=4, blocking=False)
    pclConfIn = device.getInputQueue(name="config", maxSize=4, blocking=False)
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0,0,0])
    vis.add_geometry(coordinateFrame)
    vis.add_geometry(pcd)

    first = True
    rot = 0
    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)
    while device.isPipelineRunning():
        
        inMessage = q.get()
        inColor = inMessage["rgb"]
        inPointCloud = inMessage["pcl"]
        cvColorFrame = inColor.getCvFrame()
        depth_q = depthQueue.get()
        inDet = detectionNNQueue.get()
        
        cvRGBFrame = cv2.cvtColor(cvColorFrame, cv2.COLOR_BGR2RGB)
        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        # frame = inPreview.getCvFrame()
        cvRGBFrame = cv2.cvtColor(cvColorFrame, cv2.COLOR_BGR2RGB)
        frame=cvRGBFrame


        depthFrame = depth_q.getFrame() # depthFrame values are in millimeters

        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        for detection in detections:
            roiData = detection.boundingBoxMapping
            roi = roiData.roi
            roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
            topLeft = roi.topLeft()
            bottomRight = roi.bottomRight()
            xmin = int(topLeft.x)
            ymin = int(topLeft.y)
            xmax = int(bottomRight.x)
            ymax = int(bottomRight.y)
            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)

            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = labelMap[detection.label]
            except:
                label = detection.label
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            shape_data = json.dumps({"shape": label, "x": detection.spatialCoordinates.x, "y": detection.spatialCoordinates.y,"z": detection.spatialCoordinates.z }).encode()
            sock.send_multipart([b"shape", shape_data])
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
        cv2.imshow("depth", depthFrameColor)
        cv2.imshow("preview", frame)

        if cv2.waitKey(1) == ord('q'):
            break


        if inPointCloud:
            points = inPointCloud.getPoints().astype(np.float64)
            # print(points)
            distance_threshold = 1000  # in mm

            # Compute Euclidean distance from the origin
            distances = np.linalg.norm(points, axis=1)  # Computes sqrt(x^2 + y^2 + z^2) for each point
            filtered_points=points
            mask = (points[:, 2] >= 0) & (points[:, 2] <= 800)  # Keep only points within 0-40 cm
            filtered_points = points[mask]
            # Apply the filtered points to the Open3D point cloud
            pcd.points = o3d.utility.Vector3dVector(filtered_points)

            height, width, _ = cvRGBFrame.shape  # Get RGB frame size
            depth_height, depth_width = depthFrame.shape  # Get depth frame size

            # Resize RGB to match depth frame
            if (height, width) != (depth_height, depth_width):
                cvRGBFrame = cv2.resize(cvRGBFrame, (depth_width, depth_height))

            # Now reshape safely
            colors = (cvRGBFrame.reshape(-1, 3) / 255.0).astype(np.float64)



            filtered_colors = colors[mask]  # Apply the same mask to colors

            pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
          
            # pcd = pcd.voxel_down_sample(voxel_size=2.0)  # Change 2.0 to a higher number for more downsampling

            # Convert Open3D PointCloud to NumPy array
            points = np.asarray(pcd.points, dtype=np.float32)  # Shape: (N, 3)
            colors = np.asarray(pcd.colors, dtype=np.float32)  # Shape: (N, 3)

            # Stack XYZ and RGB together
            point_cloud_with_color = np.hstack((points, colors))  # Shape: (N, 6)

            # Convert to binary format
            serialized_data = struct.pack(f"{point_cloud_with_color.size}f", *point_cloud_with_color.flatten())

            sock.send_multipart([b"pointcloud", serialized_data])

            print(f"Sent {len(points)} points with color")

            if first:

                vis.add_geometry(pcd)
                first = False
            else:
                vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
            
    vis.destroy_window()
    cv2.destroyAllWindows()

while True:
    try:
        sock.send_string("joints_str")  # Send message
        print("Sent: joints_str")
        time.sleep(1)  # Avoid spamming
    except zmq.ZMQError as exc:
        print(f"ZMQ Error: {exc}")
        break
