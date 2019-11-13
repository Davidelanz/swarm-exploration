#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <iostream>
#include <std_msgs/Float64.h>
#define LIMIT 5

// Globals
ros::Publisher pub;
geometry_msgs::Pose robotPose;
int map[2 * LIMIT + 1][2 * LIMIT + 1] = {0};

// Callbacks
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robotPose = msg->pose.pose;
}

bool inNode(double robotX, double robotY, int &nearestNodeX, int &nearestNodeY)
{
    // Find nearest node
    nearestNodeX = round(robotX);
    nearestNodeY = round(robotY);
    // ROS_INFO("Nearest node: (%d , %d)",nearestNodeX,nearestNodeY);
    // Check if I'm close enough
    double dist = sqrt(pow(robotX - nearestNodeX, 2) + pow(robotY - nearestNodeY, 2));
    // ROS_INFO("Distance from node: %lf",dist);
    return (dist <= 0.2);
}

bool updateDesPos(ros::NodeHandle nh, std::string &direction, int nearestNodeX, int nearestNodeY)
{
    // Go up and change the direction of the movement
    if (direction == "right")
    {
        // If you can go right
        if(nearestNodeX != LIMIT)
        {
            ROS_INFO("Space available along X");
            nh.setParam("des_pos_x", nearestNodeX + 1);
            nh.setParam("des_pos_y", nearestNodeY);
        }
        else // You have to go up of 1 along y
        {
            ROS_INFO("Map limit reached");
            nh.setParam("des_pos_x", nearestNodeX);
            nh.setParam("des_pos_y", nearestNodeY + 1);
            // Now you will move along x from right to left
            direction = "left";
        }
    } 
    else if (direction == "left")
    {
        // If you can go right
        if(nearestNodeX != -LIMIT)
        {
            ROS_INFO("Space available along X");
            nh.setParam("des_pos_x", nearestNodeX - 1);
            nh.setParam("des_pos_y", nearestNodeY);
        }
        else // You have to go up of 1 along y
        {
            ROS_INFO("Map limit reached");
            nh.setParam("des_pos_x", nearestNodeX);
            nh.setParam("des_pos_y", nearestNodeY + 1);
            // Now you will move along x from right to left
            direction = "right";
        }
    } 
    return true;
}


// Main
int main(int argc, char **argv)
{

    // Node Initialization
    ros::init(argc, argv, "basic_planner");
    ros::NodeHandle nh;
    
    // Get spawn position
    double X_spawn, Y_spawn;
    nh.getParam("spawnX", X_spawn);
    nh.getParam("spawnY", Y_spawn);

    // Subscribers & Services Clients
    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);
    
    // Publishers & Services Servers
    // ...

    // Loop variables
    double robotX, robotY;
    int nearestNodeX, nearestNodeY;
    int lastNodeVisitedX = X_spawn;
    int lastNodeVisitedY = Y_spawn;
    bool newNode = false;
    bool inInitialPos =  false;
    std::string direction = "right";

    ros::Rate rate(10);
    while (ros::ok())
    {
        // Perform callbacks
        ros::spinOnce();

        // Store current position
        robotX = robotPose.position.x;
        robotY = robotPose.position.y;
        // ROS_INFO("Current position: (%lf , %lf)",robotX,robotY);

        // Check if we are in a new node
        if (inNode(robotX, robotY, nearestNodeX, nearestNodeY))
        {
            newNode =  ((lastNodeVisitedX != nearestNodeX) || (lastNodeVisitedY != nearestNodeY));
            lastNodeVisitedX = nearestNodeX;
            lastNodeVisitedY = nearestNodeY;
        }

        // Check if position is reached
        inInitialPos = (nearestNodeX == -LIMIT || nearestNodeY == -LIMIT);
        if(!inInitialPos)
        {
            // If initialPos is not reached keep moving
            // towards (-LIMIT,-LIMIT)
            // ROS_INFO("Changing desired position towards (%d, %d)", -LIMIT,-LIMIT);
            nh.setParam("des_pos_x", -LIMIT); 
            nh.setParam("des_pos_y", -LIMIT);
            continue;
        }

        // If we're not in a new node, don't change the desired position
        if(newNode) 
            updateDesPos(nh, direction, nearestNodeX, nearestNodeY);

        // If you reach the last node, the trip finished
        bool inUpperLeftNode = (nearestNodeX == -LIMIT && nearestNodeY == LIMIT);
        bool inUpperRightNode = (nearestNodeX == LIMIT && nearestNodeY == LIMIT);
        if(inUpperLeftNode || inUpperRightNode)
        {
            inInitialPos = false; 
            ROS_INFO("Last node reached");
        }

        rate.sleep();
    }

    return 0;
}