#! /usr/bin/env python

import rospy
# import ros message
from std_msgs.msg import String
# plot libs
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np


# global maps
map_size = 11
node_map = np.zeros((map_size, map_size), int)


def callback(data):
    global node_map

    coded_map = data.data

    if (coded_map != ""):
        # Decode map
        tmp_map = list(coded_map.split(","))

        for i in range(0, map_size-1):
            for j in range(0, map_size-1):
                node_map[i][j] = int(tmp_map[i+j])


def main():
    global coded_map

    rospy.init_node('map_plotter')

    # Subscribe to the map
    rospy.Subscriber('node_map', String, callback)

    plt.imshow(node_map)
    plt.show()   
    
    rate.sleep()


if __name__ == "__main__":
    main()
