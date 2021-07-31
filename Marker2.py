# Stephan McGlashan
# 7/23/21
# Tello Drone ArUco Marker Class
import numpy as np


# Marker Side Length
def marker_side_length(corners):
    side_length_1 = np.sqrt(((corners[1][0]-corners[0][0])**2)+(corners[1][1]-corners[0][1])**2)  # top
    side_length_2 = np.sqrt(((corners[2][0]-corners[1][0])**2)+(corners[2][1]-corners[1][1])**2)  # _|
    side_length_3 = np.sqrt(((corners[3][0]-corners[2][0])**2)+(corners[3][1]-corners[2][1])**2)  # _
    side_length_4 = np.sqrt(((corners[0][0]-corners[3][0])**2)+(corners[0][1]-corners[3][1])**2)  # |_

    side_lengths = [side_length_1, side_length_2, side_length_3, side_length_4]
    avg_side_length = np.mean(side_lengths)
    return avg_side_length  # returns side_lengths and average


# Marker Centroid
def marker_centroid(corners):
    x_cords = (corners[0][0], corners[1][0], corners[2][0], corners[3][0])
    y_cords = (corners[0][1], corners[1][1], corners[2][1], corners[3][1])
    centroid = (int((np.mean(x_cords))), (int(np.mean(y_cords))))
    return centroid


def marker_slopes(corners):
    dy = corners[2][1]-corners[3][1]
    dx = corners[2][0]-corners[3][0]
    slope = dy/dx
    return slope


# Parameters of Interest
class Marker:

    # Default Constructor (not sure what this does)
    def __init__(self, marker_corners, marker_ids):
        self.arucoCorners = marker_corners
        self.arucoIds = marker_ids
        self.avg_side_length = marker_side_length(marker_corners[0])
        self.centroid_X = marker_centroid(marker_corners[0])[0]
        self.centroid_Y = marker_centroid(marker_corners[0])[1]
        self.slope = marker_slopes(marker_corners[0])


