# Celeritas Main Code
# Stephan McGlashan & Vaibhavi Addala
# 7/30/2021

# Imports
import numpy as np
import djitellopy as tello
from visual_odometry import PinholeCamera, VisualOdometry
import cv2
import cv2.aruco
import Marker2 as mk

# Initialization
myTello = tello.Tello()
myTello.connect()
myTello.streamon()
print("Battery: " + str(myTello.get_battery()))
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
parameters = cv2.aruco.DetectorParameters_create()
kp_yaw = 1
kp_fwd = 2
traj = np.zeros((600, 600, 3), dtype=np.uint8)
cam = PinholeCamera(960.0, 720.0, 921.170702, 919.018377, 459.904354, 351.238301, k1=-0.033458, k2=0.105152,
                    p1=0.001256, p2=-0.006647, k3=0.000000)
vo = VisualOdometry(cam)


# Course Mapping Function
def mapping(img):  # will map desired course
    course_cords_x = {}
    course_cords_y = {}
    while True:
        img_id = 0
        if img is None:  # if empty frame is returned by the camera it will be skipped
            continue
        vo.update(img, img_id)
        cur_t = vo.cur_t
        if img_id > 2:
            x, y, z = cur_t[0], cur_t[1], cur_t[2]  # returns coordinates from visual odometry code
            current_cords = [x, y, z]
            if marker_corners and marker1.avg_side_length < 500:  # if marker is detected coordinates are stored
                course_cords_x[marker_ids] = current_cords[0]
                course_cords_y[marker_ids] = current_cords[1]
        else:
            x, y, z = 0., 0., 0.
        img_id += 1

    cv2.imwrite('coursemap.png', traj)
    return course_cords_x, course_cords_y  # returns dictionaries of X and Y coordinates for each marker detected


# Main Loop
while True:
    image = cv2.cvtColor(myTello.get_frame_read().frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=parameters)

    if marker_corners:  # if marker is detected
        marker1 = mk.Marker(marker_corners[0], marker_ids[0])
        image = cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)  # draws ID and corners
        cv2.circle(image, (marker1.centroid_X, marker1.centroid_Y), 3, (0, 255, 0), cv2.FILLED)  # draws centroid

    course_cords_x, course_cords_y = mapping(image)  # retrieves marker coordinates

    '''for i, k in course_cords_x, course_cords_y:  # iterates through marker coordinates
        if i or k > len(course_cords_x)-1:
            i = 0
            k = 0'''
    i = 0
    while i <= len(course_cords_y)-1:
        slope = ((course_cords_y[i+1] - course_cords_y[i]) / (course_cords_x[i+1] - course_cords_x[i]))
        while slope < -0.3:  # PID to navigate between markers
            myTello.send_rc_control(0, 0, 0, kp_yaw)
            kp_yaw /= 1.5
        while slope > 0.3:
            myTello.send_rc_control(0, 0, 0, -kp_yaw)
            kp_yaw /= 1.5
        if abs(slope) < 0.3:
            myTello.send_rc_control(0, kp_fwd, 0, 0)
        i += 1
        if i == len(course_cords_y)-1:  # if the end of the course is reached begin another lap
            i = 0
    cv2.imshow("output", image)
    cv2.waitKey(1)


