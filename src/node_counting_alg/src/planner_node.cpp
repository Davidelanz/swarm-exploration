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
#include <numeric>
#include <vector>
#include <random>
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

bool updateDesPos(ros::NodeHandle nh, int nearestNodeX, int nearestNodeY, int robot_ID)
{
    // Array of possibile moves alongside all 8 direction                                                     //When the robot is not in "RandomMovement" mode
    vector<vector<int>> move{{+0, +1},
                             {+1, +0},
                             {+0, -1},
                             {-1, +0},
                             {+1, +1},
                             {+1, -1},
                             {-1, -1},
                             {-1, +1}};
    //the priority order to move in a unvisited node is:
    // up > right > down > left >   up_right > down_right > down_left > up_left
    // ^-<orthogonal movements>-^   ^----------<diagonal movements>-----------^

    //random movement flag
    bool randomMovement;
    nh.getParam("random", randomMovement);
    // put a high count minimum
    int min = 99999;
    // index of desired position
    int desIdx = 8;

    printMap();

    vector<int> index = {0, 1, 2, 3, 4, 5, 6, 7};

    if (randomMovement)
    {
        vector<int> randIndex = {0, 1, 2, 3};
        //even while randomly moving,the orthogonal moves has priority on the diagonal moves
        random_shuffle(randIndex.begin(), randIndex.end());
        //shuffling the first 4 entries
        vector<int> randIndexDiag = {4, 5, 6, 7};
        random_shuffle(randIndexDiag.begin(), randIndexDiag.end());
        //shuffling the last 4 entries

        randIndex.insert(randIndex.end(), randIndexDiag.begin(), randIndexDiag.end());
        //concatenating the two vectors
        for (int i = 0; i < 8; i++)
        {
            index.at(i) = randIndex.at(i);
        }
    }

    for (int i = 0; i < 8; i++)
    {
        try
        {
            int next_node_X = nearestNodeX + LIMIT + move.at(index.at(i)).at(0);
            int next_node_Y = nearestNodeY + LIMIT + move.at(index.at(i)).at(1);
            int curr_count = node_map.at(next_node_X).at(next_node_Y);

            if (curr_count < 0)
            {
                // It's an obstacle
                continue;
            }
            else if (curr_count == 0)
            {
                // If the next node is 0 just go there
                desIdx = i;
                break;
            }
            else if (curr_count <= min)
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

        int new_X = nearestNodeX + move.at(desIdx).at(0);
        int new_Y = nearestNodeY + move.at(desIdx).at(1);
        ROS_INFO("Updated position for Robot %d: (%d,%d)", robot_ID, new_X, new_Y);
        nh.setParam("des_pos_x", new_X);
        nh.setParam("des_pos_y", new_Y);
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
    srand(time(NULL));
    ros::init(argc, argv, "basic_planner");
    ros::NodeHandle nh;

    int robot_ID;
    nh.getParam("robot_ID", robot_ID);
    int initX, initY; // initial position
    nh.getParam("dex_pos_x", initX);
    nh.getParam("des_pos_y", initY);

    // Subscribers & Services Clients
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber map_sub = nh.subscribe("/node_map", 1000, mapCallback);
    // Publishers & Services Servers
    ros::Publisher visitedPub;
    visitedPub = nh.advertise<geometry_msgs::Point32>("/node_visited", 5);

    // Loop variables
    double robotX, robotY, robotYaw, lastRobotX, lastRobotY, lastRobotYaw;
    int nearestNodeX, nearestNodeY;
    int lastNodeVisitedX = map_size * map_size;
    int lastNodeVisitedY = map_size * map_size;
    bool newNode = false;
    string direction = "right";
    geometry_msgs::Point32 node_visited;

    ros::Rate rate(50);

    // Approaching to the map phase: reach initial position
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

        // If we're not in the desired intial position, keep moving towards it
        if (nearestNodeX != initX && nearestNodeY != initY)
            break;

        rate.sleep();
    }

    // Exploration phase
    while (ros::ok())
    {
        // Perform callbacks
        ros::spinOnce();

        // Store current position
        robotX = roundf(robotPose.position.x * 100) / 100;
        robotY = roundf(robotPose.position.y * 100) / 100;
        /* float roll  = robotPose.orientation.x;
        float pitch = robotPose.orientation.y;
        float yaw   = robotPose.orientation.z;
        robotYaw = roundf(q.toRotationMatrix().eulerAngles(roll, pitch, yaw)*100)/100;
        */
        robotYaw = robotPose.orientation.z;
        // ROS_INFO("Current position: (%lf , %lf)",robotX,robotY);

        // Check if we moved
        bool notMoving = (robotX == lastRobotX && robotY == lastRobotX && robotYaw == lastRobotYaw);
        // Then update last exact position
        lastRobotX = robotX;
        lastRobotY = robotY;
        lastRobotYaw = robotYaw;

        // Check if we are in a new node
        if (inNode(robotX, robotY, nearestNodeX, nearestNodeY))
        {
            newNode = ((lastNodeVisitedX != nearestNodeX) || (lastNodeVisitedY != nearestNodeY));
            lastNodeVisitedX = nearestNodeX;
            lastNodeVisitedY = nearestNodeY;
        }

        // Check if we are in the map
        if (abs(robotX) >= LIMIT + .5 || abs(robotY) >= LIMIT + .5)
        {
            ROS_INFO("Robot %d out of the map \n", robot_ID);
            // go towards the origin
            nh.setParam("des_pos_x", 0);
            nh.setParam("des_pos_y", 0);
            continue;
        }

        // If we are in a new node in the map, publish the position of the node we just entered
        if (newNode)
        {
            updateDesPos(nh, nearestNodeX, nearestNodeY, robot_ID);
            node_visited.x = nearestNodeX;
            node_visited.y = nearestNodeY;
            node_visited.z = robot_ID;
            visitedPub.publish(node_visited);
        }

        // If we are not moving, just update the desired position
        if (notMoving)
        {
            ROS_INFO("Robot %d not moving", robot_ID);
            updateDesPos(nh, nearestNodeX, nearestNodeY, robot_ID);
        }

        rate.sleep();
    }

    return 0;
}