import cv2
import depthai as dai
import numpy as np
from collections import deque


# Get camera intrinsics from board
# pass id for RGB, LEFT, and RIGHT cameras
# also called CAM_A, CAM_B, CAM_C
def get_intrinsics(camera_board):
    with dai.Device() as device:
        # Read calibration data from EEPROM
        calibData = device.readCalibration()
        
        # Get intrinsics for the Left camera at a specific resolution
        intrinsics = calibData.getCameraIntrinsics(
            camera_board, 
            resizeWidth=640, 
            resizeHeight=400
        )

        dist_coeffs = np.array(
            calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        )

        return intrinsics, dist_coeffs

# Set intrinsics from camera board
def set_intrinsics(camera_matrix, dist_coeffs):
    
    new_camera_matrix = np.array([
        camera_matrix[0],
        camera_matrix[1],
        camera_matrix[2]
    ], dtype=np.float32)

    dist_coeffs = dist_coeffs

    return new_camera_matrix, dist_coeffs

# Creates a pipeline for RGB and stereo pointcloud outputs
# returns -1 is pipeline creation fails
def create_pipeline():
    with dai.Pipeline() as p:
        
        ### FROM LUXONIS CHAT
        rgbd = p.create(dai.node.RGBD).build()
        stereo = p.create(dai.node.StereoDepth)
        color = p.create(dai.node.Camera).build()
        left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        left.requestOutput((640, 400)).link(stereo.left)
        right.requestOutput((640, 400)).link(stereo.right)

        platform = p.getDefaultDevice().getPlatform()

        if platform == dai.Platform.RVC4:
            out = color.requestOutput((640,400), dai.ImgFrame.Type.RGB888i, enableUndistortion=True)
            align = p.create(dai.node.ImageAlign)
            stereo.depth.link(align.input)
            out.link(align.inputAlignTo)
            align.outputAligned.link(rgbd.inDepth)
        else:
            out = color.requestOutput((640,400), dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 30, True)
            stereo.depth.link(rgbd.inDepth)
            out.link(stereo.inputAlignTo)   
        out.link(rgbd.inColor)
        ### end
 
        qPcl   = rgbd.pcl.createOutputQueue()
        qRgb   = rgbd.rgbd.createOutputQueue()
        qDepth = stereo.depth.createOutputQueue()
    
        return p, (qPcl, qRgb, qDepth)

    return -1

def colorizeDepth(frameDepth: np.ndarray) -> np.ndarray:
    invalidMask = frameDepth == 0  # zero depth is invalid

    try:
        # Compute 3rd and 95th percentiles over valid (non-zero) depths
        valid = frameDepth[frameDepth != 0]
        minDepth = np.percentile(valid, 3)
        maxDepth = np.percentile(valid, 95)

        # Log of depth, only where depth != 0
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)

        # Replace NaNs (from log(0)) with minimum log depth
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)

        # Clip to [logMinDepth, logMaxDepth]
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        # Normalize to [0, 255]
        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)

        # Apply JET colormap
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)

        # Set invalid depth pixels to black
        depthFrameColor[invalidMask] = 0
    except IndexError:
        # Frame is likely empty
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e

    return depthFrameColor

    


               