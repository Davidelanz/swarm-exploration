#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include <std_msgs/Float64.h>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <algorithm>
using namespace std;
#define LIMIT 5

// Globals
ros::Publisher pub;
geometry_msgs::Pose robotPose;
const int map_size = 2 * LIMIT + 1;
vector<vector<int>> node_map(map_size, vector<int>(map_size, 0));

// Prototypes
vector<vector<int>> mapDecoder(string s);

// Callbacks
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robotPose = msg->pose.pose;
}

void mapCallback(const std_msgs::String::ConstPtr &msg)
{
    node_map = mapDecoder(msg->data);
}

// Functions

void printMap()
{
    for (int y = map_size - 1; y >= 0; y--)
    {
        for (int x = 0; x < map_size; x++)
            cout << node_map.at(x).at(y) << " ";
        cout << endl;
    }
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

bool updateDesPos(ros::NodeHandle nh, int nearestNodeX, int nearestNodeY)
{
    // Array of possibile moves alongside all 8 direction
    //vector<vector<int>> move{{+0, +1}, {+1, +1}, {+1, 0}, {+1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, +1}};
    vector<vector<int>> move{{+0, +1},{+1, +0},{+0, -1},{-1, +0},{ +1, +1},{+1, -1},{-1,  -1},{-1, +1}}; //the priority order to move in a unvisited node is:
                                                                                                        // up > right > down > left >   up_right > down_right > down_left > up_left
                                                                                                        // ^-<orthogonal movements>-^   ^----------<diagonal movements>-----------^


                                                                                            
    // put a high count minimum                                                                            
    int min = 99999;
    // index of desired position
    int desIdx = 8;

    printMap();

    for (int i = 0; i < 8; i++)
    {
        try
        {
            int next_idx_X = nearestNodeX + LIMIT + move.at(i).at(0);
            int next_idx_Y = nearestNodeY + LIMIT + move.at(i).at(1);
            int curr_count = node_map.at(next_idx_X).at(next_idx_Y);
            ROS_INFO("Count value for node (%d,%d) : %d - (Min value : %d)", next_idx_X, next_idx_Y, curr_count, min);

            if (curr_count == 0)
            {
                desIdx = i;
                break;
            }
            else if (curr_count < min)
            {
                min = curr_count;
                desIdx = i;
            }
        }
        catch (const std::exception &e)
        {
            ROS_INFO("Near map bounds");
        }
    }
    if (desIdx < 8 || desIdx >= 0)
    {
        nh.setParam("des_pos_x", nearestNodeX + move.at(desIdx).at(0));
        nh.setParam("des_pos_y", nearestNodeY + move.at(desIdx).at(1));
        return true;
    }
    else
    {
        ROS_ERROR("Update position failed: desired index not correct");
        return false;
    }
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

    int robot_ID;
    nh.getParam("robot_ID", robot_ID);

    // Subscribers & Services Clients
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber map_sub = nh.subscribe("/node_map", 1000, mapCallback);
    // Publishers & Services Servers
    ros::Publisher visitedPub;
    visitedPub = nh.advertise<geometry_msgs::Point32>("/node_visited", 5);

    // Loop variables
    double robotX, robotY;
    int nearestNodeX, nearestNodeY;
    int lastNodeVisitedX = map_size * map_size;
    int lastNodeVisitedY = map_size * map_size;
    bool newNode = false;
    bool inInitialPos = false;
    string direction = "right";
    geometry_msgs::Point32 node_visited;

    ros::Rate rate(50);
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

        // If we're not in a new node, don't change the desired position
        if (newNode)
        {
            updateDesPos(nh, nearestNodeX, nearestNodeY);
            node_visited.x = nearestNodeX;
            node_visited.y = nearestNodeY;
            node_visited.z = robot_ID;
            visitedPub.publish(node_visited);
        }

        rate.sleep();
    }

    return 0;
}
