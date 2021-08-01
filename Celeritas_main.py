# Celeritas Main Code
# Stephan McGlashan & Vaibhavi Addala
# 7/30/2021

# Imports
import numpy as np
import djitellopy as tello
import cv2.aruco
from visual_odometry import PinholeCamera, VisualOdometry
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
obstacle_aruco = 7
marker_aruco = 9


# Course Mapping Function
def mapping():  # will map desired course
    mapping_done = False

    course_cords_x = {}
    course_cords_y = {}
    img_id = 0
    i = 0
    n = 0
    while not mapping_done:
        image = myTello.get_frame_read().frame
        img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=parameters)
        if marker_corners:  # if marker is detected
            print("marker")
            marker1 = mk.Marker(marker_corners[0], marker_ids[0])
            image = cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)  # draws ID and corners
            cv2.circle(image, (marker1.centroid_X, marker1.centroid_Y), 3, (0, 255, 0), cv2.FILLED)  # draws centroid
        if img is None:  # if empty frame is returned by the camera it will be skipped
            print("frame dropped")
        else:
            print(img.shape)
            vo.update(img, img_id)
            cur_t = vo.cur_t
            if img_id > 2:
                x, y, z = cur_t[0], cur_t[1], cur_t[2]  # returns coordinates from visual odometry code
                current_cords = [x, y, z]
                if marker_corners and marker1.avg_side_length < 500:  # if marker is detected coordinates are stored
                    course_cords_x[n] = current_cords[0]
                    course_cords_y[n] = current_cords[1]
                    print("Coordinates for Marker ID: " + str(marker_ids) + " have been logged!")  # confirmation
                    n += 1  # counter for number of markers
                elif marker_corners and marker1.avg_side_length > 500:
                    print("Marker ID: " + str(marker_ids) + " is not close enough!")
                else:
                    print("no marker detected")
                    continue
            else:
                x, y, z = 0., 0., 0.

            draw_x, draw_y = int(x) + 290, int(z) + 90  # displays plotted course
            cv2.circle(traj, (draw_x, draw_y), 1, (img_id * 255 / 4540, 255 - img_id * 255 / 4540, 0), 1)
            cv2.rectangle(traj, (10, 20), (600, 60), (0, 0, 0), -1)
            text = "Coordinates: x=%2fm y=%2fm z=%2fm" % (x, y, z)
            cv2.putText(traj, text, (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
            cv2.imshow('Trajectory', traj)
            cv2.waitKey(1)
            img_id += 1
            i += 1
            print("Frame: " + str(i))
            if marker_ids == 24:  # end checkpoint
                print("Mapping has been completed!")
                mapping_done = True


    cv2.imwrite('coursemap.png', traj)
    return course_cords_x, course_cords_y  # returns dictionaries of X and Y coordinates for each marker detected


# Main Loop
course_cords_x, course_cords_y = mapping()  # retrieves marker coordinates
while True:
    print("sequence initiated")
    img = myTello.get_frame_read().frame
    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=parameters)

    if marker_corners:  # if marker is detected
        print("marker")
        marker1 = mk.Marker(marker_corners[0], marker_ids[0])
        image = cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)  # draws ID and corners
        cv2.circle(image, (marker1.centroid_X, marker1.centroid_Y), 3, (0, 255, 0), cv2.FILLED)  # draws centroid

    myTello.takeoff()
    lap_count = 3
    laps_done = 0
    i = 0
    print("navigation sequence initiated")
    while i <= len(course_cords_y) - 1:
        if i == len(course_cords_y) - 1:  # if the end of the course is reached begin another lap
            slope = ((course_cords_y[0] - course_cords_y[i]) / (course_cords_x[0] - course_cords_x[i]))
        else:
            slope = ((course_cords_y[i + 1] - course_cords_y[i]) / (course_cords_x[i + 1] - course_cords_x[i]))
        # PID to navigate between markers
        while slope < -0.3:  # adjustments for negative slope
            myTello.send_rc_control(0, 0, 0, int(kp_yaw))
            kp_yaw /= 2
        while slope > 0.3:  # adjustments for positive slope
            myTello.send_rc_control(0, 0, 0, int(-kp_yaw))
            kp_yaw /= 2
        if abs(slope) < 0.3:  # flies forward with correct slope
            myTello.send_rc_control(0, int(kp_fwd), 0, 0)
            if marker_ids == obstacle_aruco:  # an obstacle has been detected
                myTello.move_right(30)
                myTello.move_forward(30)
                myTello.move_left(30)
        i += 1
        if i == len(course_cords_y) - 1:  # if the end of the course is reached begin another lap
            i = 0
            laps_done += 1
            if laps_done == lap_count:
                myTello.land()
                print("Course complete, after " + str(laps_done) + " laps!")
    cv2.imshow("output", image)
    cv2.waitKey(1)
