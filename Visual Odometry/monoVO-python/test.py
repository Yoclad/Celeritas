import numpy as np
import cv2
import time
import traceback
from visual_odometry import PinholeCamera, VisualOdometry
from djitellopy import Tello

tello = Tello()
tello.connect()
tello.streamon()
cam = PinholeCamera(960.0, 720.0, 921.170702, 919.018377, 459.904354, 351.238301, k1=-0.033458, k2=0.105152,
                    p1=0.001256, p2=-0.006647, k3=0.000000)
vo = VisualOdometry(cam)
traj = np.zeros((600, 600, 3), dtype=np.uint8)
img_id = 0
tello.takeoff()
time.sleep(2)
while True:
    try:
        image = tello.get_frame_read().frame
        if image is None:
            continue
        # Convert frame to grayscale
        img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        vo.update(img, img_id)

        cur_t = vo.cur_t
        if img_id > 2:
            x, y, z = cur_t[0], cur_t[1], cur_t[2]
        else:
            x, y, z = 0., 0., 0.
        draw_x, draw_y = int(x) + 290, int(z) + 90

        cv2.circle(traj, (draw_x, draw_y), 1, (img_id * 255 / 4540, 255 - img_id * 255 / 4540, 0), 1)
        cv2.rectangle(traj, (10, 20), (600, 60), (0, 0, 0), -1)
        text = "Coordinates: x=%2fm y=%2fm z=%2fm" % (x, y, z)
        cv2.putText(traj, text, (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)

        cv2.imshow('Forward facing camera', img)
        cv2.imshow('Trajectory', traj)
        cv2.waitKey(1)
        img_id += 1
    except Exception as e:
        #print(traceback.print_exc())
        pass


cv2.imwrite('map.png', traj)
