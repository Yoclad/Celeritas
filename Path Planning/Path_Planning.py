import math
import scipy as sp
import networkx as nx
import djitellopy as tello
import time
import cv2
import cv2.aruco

myTello = tello.Tello()
myTello.connect()
myTello.streamon()

# G=nx.fast_gnp_random_graph(5, 1)
# adj_matrix = nx.adjacency_matrix(G)
# testing
G = nx.Graph()
now = 0
dest = 1
kp_yaw = 1
kp_fwd = 2

current = G.nodes[now]  # this will be the starting node
neighbor = G.nodes[dest]  # next node

while True:
    current_cords = current['pos']  # UPDATE TO DRONE'S POSITION. NOT NODE'S.
    neighbor_cords = neighbor['pos']  # next node's coordinates
    slope = ((neighbor_cords[1] - current_cords[1])/(neighbor_cords[0] - current_cords[0]))  # slope to neighbor

# PID stuff:
    while slope < -0.3:
        myTello.send_rc_control(0, 0, 0, kp_yaw)
        kp_yaw /= 1.5
    while slope > 0.3:
        myTello.send_rc_control(0, 0, 0, -kp_yaw)
        kp_yaw /= 1.5
    if abs(slope) < 0.3:
        myTello.get_frame_read()
        img = myTello.background_frame_read.frame
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        pars = cv2.aruco.DetectorParameters_create()
        corners, ids, unused = cv2.aruco.detectMarkers(img, dictionary, parameters=pars)
        if len(corners) == 0:
            myTello.send_rc_control(0, kp_fwd, 0, 0)
        else:
            # we've arrived at neighbor
            if len(G.neighbors(current)) == 0:
                myTello.land()  # no neighbors means we're at the end of the course
            else:
                G.remove_node(current)
                current = neighbor  # switch current to node we're currently on (neighbor)
                neighbor = G.neighbors(neighbor)
                # since we've deleted the other neighbor (current) it can only access the next node

# Pseudo Code:
# while flight_not_done:
# access the coordinates of current node and neighbor node, then calculate slope and distance
# use PID to ensure you're on the correct slope for the correct amount of time (or until a marker is detected).
# if this current node has a neighbor, move to that neighbor and mark the current as 'visited'
# node['pos']

# Unnecessary:
# dist = math.dist(neighbor_cords, current_cords)  # distance between the coordinates - may not be needed
