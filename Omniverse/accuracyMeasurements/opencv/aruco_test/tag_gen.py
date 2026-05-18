import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

marker_id = 0
marker_size = 300

marker_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite("aruco_marker_0.png", marker_img)
cv2.imshow("ArUco Marker", marker_img)
cv2.waitKey(0)
cv2.destroyAllWindows()