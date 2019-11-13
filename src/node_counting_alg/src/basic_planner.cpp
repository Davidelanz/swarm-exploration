#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <cmath>
#include <iostream>
#include <cstdio>
using namespace std;
#define LIMIT 5

// Globals
ros::Publisher pub;
geometry_msgs::Pose robotPose;
const int map_size = 2 * LIMIT + 1;
vector<vector<int>> node_map(map_size, vector<int>(map_size, 0));
// Callbacks
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robotPose = msg->pose.pose;
}

void mapCallback(const std_msgs::String::ConstPtr &msg)
{
    node_map = mapDecoder(msg->data);
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

bool updateDesPos(ros::NodeHandle nh, string &direction, int nearestNodeX, int nearestNodeY)
{
    // Array of possibile movements alongside all 8 direction
    int movement[8][2] = { {0, 1},{1, 1},{1, 0},{+1, -1},{-1, 0},{-1, -1},{-1, 0},{-1, +1} };
    // Array of reachable position with the above movements
    int min,minIndex, temp;

    for(int i = 0; i < 9; i++)
    {
        try
        {
            temp = node_map.at(nearestNodeX+ movement[i][0]).at(nearestNodeY+movement[i][1]);
            if(temp < min ) 
            {
                min = temp;
                minIndex = i;
            }

        }
        catch (const std::exception &e)
        {
            continue;
        }
    }
    nh.setParam("des_pos_x", nearestNodeX + movement[minIndex][0]);
    nh.setParam("des_pos_y", nearestNodeY) + movement[minIndex][1] ;
    

    /*
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
    } */
    return true;
}

vector<vector<int>> mapDecoder(string s)
{
    vector<vector<int>> decodedMap(map_size, vector<int>(map_size, 0));
    string delimiter = ",";

    size_t pos = 0;
    int i = 0;
    string token;

    // ROS_INFO("I'm starting to decode the map");
    while ((pos = s.find(delimiter)) != string::npos)
    {
        token = s.substr(0, pos);
        decodedMap.at(i % map_size).at(i / map_size) = stoi(token);
        s.erase(0, pos + delimiter.length());
        i++;
    }

    // ROS_INFO("Finished to decode the map");
    return decodedMap;
}

// Main
int main(int argc, char **argv)
{

    // Node Initialization
    ros::init(argc, argv, "basic_planner");
    ros::NodeHandle nh;

    // Subscribers & Services Clients
    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber sub2 = nh.subscribe("node_map", 1000, mapCallback);
    // Publishers & Services Servers
    // ...

    // Loop variables
    double robotX, robotY;
    int nearestNodeX, nearestNodeY;
    int lastNodeVisitedX = NULL;
    int lastNodeVisitedY = NULL;
    bool newNode = false;
    bool inInitialPos = false;
    string direction = "right";

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
            newNode = ((lastNodeVisitedX != nearestNodeX) || (lastNodeVisitedY != nearestNodeY));
            lastNodeVisitedX = nearestNodeX;
            lastNodeVisitedY = nearestNodeY;
        }

        /*  //go to an init position specified by X,Y
        // Check if position is reached
        inInitialPos = (nearestNodeX == X || nearestNodeY == Y);
        if(!inInitialPos)
        {
            // If initialPos is not reached keep moving
            // towards (-X,-Y)

            // ROS_INFO("Changing desired position towards (%d, %d)", -X,-Y);
            nh.setParam("des_pos_x", -X); 
            nh.setParam("des_pos_y", -Y);
            continue;
        }
        */

        // If we're not in a new node, don't change the desired position
        if (newNode)
            updateDesPos(nh, direction, nearestNodeX, nearestNodeY);

        /* End Check
        // If you reach the last node, the trip finished
        bool inUpperLeftNode = (nearestNodeX == -LIMIT && nearestNodeY == LIMIT);
        bool inUpperRightNode = (nearestNodeX == LIMIT && nearestNodeY == LIMIT);
        if(inUpperLeftNode || inUpperRightNode)
        {
            inInitialPos = false; 
            ROS_INFO("Last node reached");
        }
        */
        rate.sleep();
    }

    return 0;
}
