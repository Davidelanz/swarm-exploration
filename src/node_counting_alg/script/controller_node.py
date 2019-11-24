#! /usr/bin/env python

import rospy
# import ros message
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *

# other imports
import math

# _____________________________________________________________________

srv_client_go_to_point_ = None
position_ = Point()
yaw_ = 0
state_ = 0
state_desc_ = ['Go to node', 'Obstacle avoidance']

regions_ = {
    'right': 10,
    'fright': 10,
    'front': 10,
    'fleft': 10,
    'left': 10,
}

# _____________________________________________________________________
# callbacks


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }


def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

# _____________________________________________________________________
# functions


def change_state(state):
    global state_, state_desc_, robot_ID 
    global srv_client_go_to_point_
    state_ = state
    rospy.loginfo("Controller node - Robot %s - State [%s] %s", robot_ID, state, state_desc_[state])
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        #resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        #resp = srv_client_wall_follower_(True)
    


# _____________________________________________________________________

def main():
    global regions_, position_, state_, yaw_, robot_ID 
    global srv_client_go_to_point_

    rospy.init_node('robot_control')
    rospy.loginfo("Robot controller node started in state: %s" %
                  state_desc_[state_])

    # Get params
    robot_ID = rospy.get_param("robot_ID")

    #Subs
    sub_laser = rospy.Subscriber('front_laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)

    # Client services
    srv_client_go_to_point_ = rospy.ServiceProxy('go_to_point_switch', SetBool)

    # Pubs
    obst_pub = rospy.Publisher('/obstacle_node', Point, queue_size=10)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Initialize
    rospy.loginfo("Controller node - Waiting for service")
    rospy.wait_for_service('go_to_point_switch')
    change_state(0)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        if state_ == 0:
            # whilegoint to point if detects an obstacle
            if regions_["front"] < 0.25 or regions_["front"] > 99999:
                rospy.loginfo("Controller node - Obstacle detected")
                msg = Point()
                # Estimate obstacle's position w.r.t fixed-tobot frame transformation
                msg.x = rospy.get_param('des_pos_x') 
                msg.y = rospy.get_param('des_pos_y')
                msg.z = int(robot_ID)
                obst_pub.publish(msg)
                msg = Twist()
                msg.linear.x = - rospy.get_param('/lin_vel')/2
                msg.angular.z = 0
                twist_pub.publish(msg)
                # Go in obstacle avoidance
                change_state(1)
            
        elif state_ == 1:

            frontFree = regions_['front'] > 0.5 and (regions_['fright'] > 0.5 or  regions_['fleft'] > 0.5)
            if not frontFree:
                rospy.loginfo("Controller node - Front not free")
                msg = Twist()
                msg.linear.x = - rospy.get_param('/lin_vel')/2
                msg.angular.z = 0
                twist_pub.publish(msg)
            else:
                change_state(0)

        rate.sleep()


if __name__ == "__main__":
    main()
