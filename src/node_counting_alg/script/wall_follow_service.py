#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import random

# other imports
import math

# Navigation velocities
linear_vel = rospy.get_param('/lin_vel')
angular_vel = rospy.get_param('/ang_vel')

# _____________________________________________________________________

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

# service switch state
active_ = False

# _____________________________________________________________________


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right': min(min(msg.ranges[  0:143]), 10),
        'fright':min(min(msg.ranges[144:287]), 10),
        'fright': min(min(msg.ranges[0:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft': min(min(msg.ranges[432:575]), 10),
        'left':  min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        rospy.loginfo('Wall follower - [%s] - %s', state, state_dict_[state])
        state_ = state


def take_action():
    global regions_
    regions = regions_

    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0

    state_description = ''
    d = .6

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)  # find wall

    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)  # turn left

    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - right'
        change_state(2)  # follow straight the wall

    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - left'
        change_state(0)  # find wall

    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and right'
        change_state(1)  # turn left

    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and left'
        change_state(1)  # turn left

    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and left and right'
        change_state(1)  # turn left

    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - left and right'
        change_state(0)  # find wall

    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    return msg


def find_wall():

    msg = Twist()

    # random values to avoid symmetric robot-robot detections and mirroring
    # for both ranges there is a little contribute in the other sense in order
    # to disrupt heavy simmetries

    die = random.randint(0, 9)

    if die > 2: #turn right  (8 out of 10)
        msg.linear.x = linear_vel*4
        msg.angular.z = - angular_vel
    elif die > 1: #turn left (1 out of 10)
        msg.linear.x = linear_vel/2
        msg.angular.z = angular_vel/2
    else: #go back left (1 out of 10)
        msg.linear.x = - linear_vel/2
        msg.angular.z = angular_vel/2
    return msg


def turn_left():
    msg = Twist()
    die = random.randint(0, 9)
    if die > 2: #turn left (8 out of 10)
        msg.linear.x = linear_vel/10
        msg.angular.z = angular_vel*10
    elif die > 1: #turn right (1 out of 10)
        msg.linear.x = linear_vel/20
        msg.angular.z = - angular_vel/2
    else: #go back right (1 out of 10)
        msg.linear.x = - linear_vel/2
        msg.angular.z = - angular_vel/2
    return msg


def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = linear_vel
    msg.angular.z = 0
    return msg

# _____________________________________________________________________


def main():
    global pub_, active_

    random.seed()

    rospy.init_node('wall_follow')
    rospy.loginfo("Wall following service node started in state: %s" %
                  state_dict_[state_])

    pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('front_laser/scan', LaserScan, clbk_laser)

    srv = rospy.Service(rospy.get_namespace() +
                        'wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
