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

bool updateCounter(double robotX, double robotY, int nearestNodeX, int nearestNodeY,
                   int &lastNodeVisitedX, int &lastNodeVisitedY)
{
    // Check if I'm in a new node
    // ROS_INFO("Last node visited (%d, %d)", lastNodeVisitedX,lastNodeVisitedY);
    bool newnode =  ((lastNodeVisitedX != nearestNodeX) || (lastNodeVisitedY != nearestNodeY));
    if (newnode)
    {
        ROS_INFO("It's a new node");
        // Updating last node visited
        lastNodeVisitedX = nearestNodeX;
        lastNodeVisitedY = nearestNodeY;
        // Increment counter on the visited node
        map[nearestNodeX + LIMIT][nearestNodeY + LIMIT]++;
        ROS_INFO("Node (%d,%d) has been visited %d times", nearestNodeX,nearestNodeY,map[nearestNodeX + LIMIT][nearestNodeY + LIMIT]);
    }
    return newnode;
}


// Main
int main(int argc, char **argv)
{

    // Node Initialization
    ros::init(argc, argv, "mapper");
    ros::NodeHandle nh;
    
    // Get spawn position
    double X_spawn, Y_spawn;
    nh.getParam("spawnX", X_spawn);
    nh.getParam("spawnY", Y_spawn);

    // Subscribers & Services Clients
    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);
    
    // Publishers & Services Servers
    // ros::Publisher mapPub;
    // mapPub = nh.advertise<std_msgs::Int32MultiArray>("map", 5);


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

        if (inNode(robotX, robotY, nearestNodeX, nearestNodeY))
            newNode = updateCounter(robotX, robotY, nearestNodeX, nearestNodeY, lastNodeVisitedX, lastNodeVisitedY);
        
        //mapPub.publish(map);

        rate.sleep();
    }

    return 0;
}