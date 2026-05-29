import cv2
import numpy as np

def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to quaternion [w, x, y, z]."""
    try:
        trace = R[0,0] + R[1,1] + R[2,2]
    except:
        print("error converting R_vec to quaternion")
        return [0,0,0,0]

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

cv_to_isaac = np.array([
    [ 0,  0,  1],   # Isaac X  =  OpenCV Z
    [-1,  0,  0],   # Isaac Y  = -OpenCV X
    [ 0, 1,  0]    # Isaac Z  = -OpenCV Y
], dtype=np.float64)

def create_aruco_detector(aruco_dict):
    ARUCO_DICT = aruco_dict

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    return aruco_detector

def get_tag_corners(size_in_meters):
    MARKER_SIZE_METERS = size_in_meters

    # 3D marker corner coordinates
    obj_points = np.array([
        [-MARKER_SIZE_METERS/2,  MARKER_SIZE_METERS/2, 0],
        [ MARKER_SIZE_METERS/2,  MARKER_SIZE_METERS/2, 0],
        [ MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
        [-MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
    ], dtype=np.float32)

    return obj_points

def draw_detection(frame, rgb_cam_matrix, rgb_dist_coeffs, rvec, tvec, MARKER_SIZE_METERS, marker_id, img_pts):
    # Draw 3D axis
    cv2.drawFrameAxes(
        frame,
        rgb_cam_matrix,
        rgb_dist_coeffs,
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

def get_center_point(img_pts):
    c = img_pts
    
    # Calculate center x and y
    center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
    center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)

    return center_x, center_y
           

def find_world_origin(rvec, tvec):
    # Convert from camera to world coords
    R_tag_in_cam, _ = cv2.Rodrigues(rvec)      
    t_tag_in_cam = tvec.flatten()      

    R_cam_in_world = R_tag_in_cam.T            
    t_cam_in_world = -R_cam_in_world @ t_tag_in_cam  

    return t_cam_in_world, R_cam_in_world

def find_tag_in_world(rvec, tvec, R_cam_in_world, t_cam_in_world):
    t_endpoint_in_cam = tvec.flatten()
    t_endpoint_in_world = (R_cam_in_world @ t_endpoint_in_cam) + t_cam_in_world 

    R_endpoint_in_cam, _ = cv2.Rodrigues(rvec) 
    R_endpoint_in_world = R_cam_in_world @ R_endpoint_in_cam

    return t_endpoint_in_world, R_endpoint_in_world

def cv_to_OVIS(rvec):
    R_isaac = cv_to_isaac @ rvec @ cv_to_isaac.T

    qw, qx, qy, qz = rotation_matrix_to_quaternion(R_isaac) 

    return (qw, qx, qy, qz)

''' worry about this later
# return the xform of the tag
# using the stereo point cloud(pcl)
def get_3d_pcl():


def get_tag_center():
'''
