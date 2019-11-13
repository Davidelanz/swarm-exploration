#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <iostream>
#include <cstdio>
using namespace std;
#define LIMIT 5

// Globals
ros::Publisher pub;
geometry_msgs::Pose robotPose;

// "vector<int>(map_size,0));" create a vector of size "map_size" with all values as 0.
// we create a vector of those vectors
int map_size = 2 * LIMIT + 1;
vector<vector<int>> node_map(map_size, vector<int>(map_size, 0));

// Callbacks
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robotPose = msg->pose.pose;
}
bool inNode(double robotX, double robotY, int &nearestNodeX, int &nearestNodeY)
{
    // Find nearest node

    //ROS_INFO("Prev Nearest node: (%d , %d)",nearestNodeX,nearestNodeY);
    nearestNodeX = round(robotX);
    nearestNodeY = round(robotY);
    //ROS_INFO("Nearest node: (%d , %d)",nearestNodeX,nearestNodeY);
    // Check if I'm close enough
    double dist = sqrt(pow(robotX - nearestNodeX, 2) + pow(robotY - nearestNodeY, 2));
    ROS_INFO("Distance from node: %lf",dist);
    return (dist <= 0.2);
}

bool updateCounter(double robotX, double robotY, int nearestNodeX, int nearestNodeY,
                   int &lastNodeVisitedX, int &lastNodeVisitedY)
{
    // Check if I'm in a new node
    // ROS_INFO("Last node visited (%d, %d)", lastNodeVisitedX,lastNodeVisitedY);
    bool newnode = ((lastNodeVisitedX != nearestNodeX) || (lastNodeVisitedY != nearestNodeY));
    if (newnode)
    {
        ROS_INFO("It's a new node");
        // Updating last node visited
        lastNodeVisitedX = nearestNodeX;
        lastNodeVisitedY = nearestNodeY;
        // Increment counter on the visited node
        node_map.at(nearestNodeX + LIMIT).at(nearestNodeY + LIMIT)++;
        //ROS_INFO("Node (%d,%d) has been visited %d times", nearestNodeX, nearestNodeY, node_map[nearestNodeX + LIMIT][nearestNodeY + LIMIT]);
    }
    return newnode;
}

std::string mapCoder()
{   
    // ROS_INFO("I'm starting to code the map");
    std::string s;
    for (int i = 0; i < map_size; i++)
    {
        for (int j = 0; j < map_size; j++)
        {
            s.append(std::to_string(node_map.at(i).at(j)));
            s.append(",");
        }
    }

    // ROS_INFO("Finished to code the map");
    return s;
}

vector<vector<int>> mapDecoder(std::string s)
{
    vector<vector<int>> decodedMap(map_size, vector<int>(map_size, 0));
    std::string delimiter = ",";
    
    size_t pos = 0;
    int i = 0;
    std::string token;

    // ROS_INFO("I'm starting to decode the map");
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        token = s.substr(0, pos);
        decodedMap.at(i%map_size).at(i/map_size) = std::stoi(token);
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
    ros::init(argc, argv, "mapper");
    ros::NodeHandle nh;

    // Get spawn position
    double X_spawn, Y_spawn;
    nh.getParam("spawnX", X_spawn);
    nh.getParam("spawnY", Y_spawn);

    // Subscribers & Services Clients
    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);

    // Publishers & Services Servers
    ros::Publisher mapPub;
    mapPub = nh.advertise<std_msgs::String>("node_map", 5);

    // Loop variables
    double robotX, robotY;
    // Initialize nearest node at -1 to avoid double visiting node (0,0)
    int nearestNodeX, nearestNodeY;
    int lastNodeVisitedX = 0;
    int lastNodeVisitedY = 0;
    bool newNode = false;
    bool inInitialPos = false;
    std::string direction = "right";
    std_msgs::String codedMap;

    ros::Rate rate(10);
    // std::cin.ignore();
    while (ros::ok())
    {
        // Perform callbacks
        ros::spinOnce();

        // Store current position
        robotX = robotPose.position.x;
        robotY = robotPose.position.y;
        ROS_INFO("Current position: (%lf , %lf)", robotX, robotY);

        if (inNode(robotX, robotY, nearestNodeX, nearestNodeY))
            newNode = updateCounter(robotX, robotY, nearestNodeX, nearestNodeY, lastNodeVisitedX, lastNodeVisitedY);

        codedMap.data = mapCoder();
        for (int i = 0; i < map_size; i++)
        {
             for (int j = 0; j < map_size; j++)
            {
            std::cout<< mapDecoder(codedMap.data).at(i).at(j);
            }
            cout << endl;
        }
        mapPub.publish(codedMap);

        rate.sleep();
    }

    return 0;
}
