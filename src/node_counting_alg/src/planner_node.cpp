#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
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
float frontLaser;
int num_robot;

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

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    frontLaser = min(*min_element(&(msg->ranges.at(0)), &(msg->ranges.at(143))), 10.0f);
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

bool inNode(double robotX, double robotY, int nearestNodeX, int nearestNodeY)
{
    // ROS_INFO("Nearest node: (%d , %d)",nearestNodeX,nearestNodeY);
    // Check if I'm close enough
    double dist = sqrt(pow(robotX - nearestNodeX, 2) + pow(robotY - nearestNodeY, 2));
    // ROS_INFO("Distance from node: %lf",dist);
    return (dist <= 0.2);
}
//go to an init position specified by des_pos_X and des_pos_Y as a parameter at launch time
bool inInitPos(ros::NodeHandle nh, int nearestNodeX, int nearestNodeY, int initX, int initY)
{
    return (nearestNodeX == initX and nearestNodeY == initY);
}

bool updateDesPos(ros::NodeHandle nh, int nearestNodeX, int nearestNodeY, int robot_ID)
{

    // Array of possibile moves alongside all 8 direction
    vector<vector<int>> move{{+0, -1},
                             {-1, +0},
                             {+0, +1},
                             {+1, +0},
                             {+1, +1},
                             {+1, -1},
                             {-1, -1},
                             {-1, +1}};
    // When the robot is not in "RandomMovement" modethe priority order to move in a unvisited node is:
    // down > left > up > right   up_right > down_right > down_left > up_left
    // ^-<orthogonal movements>-^   ^----------<diagonal movements>-----------^

    //random movement flag
    bool randomMovement;
    nh.getParam("random", randomMovement);
    // put a high count minimum
    int min = 99999;
    // index of desired position
    int desIdx = 8;

    //printMap();

    vector<int> index = {0, 1, 2, 3, 4, 5, 6, 7};
    enum class moves
    {
        down,
        left,
        up,
        right,
        up_right,
        down_right,
        down_left,
        up_left
    };
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
            //ROS_INFO("Near map bounds");
        }
    }

    if (desIdx < 8 || desIdx >= 0)
    {

        int new_X = nearestNodeX + move.at(desIdx).at(0);
        int new_Y = nearestNodeY + move.at(desIdx).at(1);
        //ROS_INFO("Updated position for Robot %d: (%d,%d)", robot_ID, new_X, new_Y);
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

int main(int argc, char **argv)
{

    // Node Initialization
    srand(time(NULL));
    ros::init(argc, argv, "basic_planner");
    ros::NodeHandle nh;

    int robot_ID;
    nh.getParam("robot_ID", robot_ID);
    int initX, initY; // initial position
    nh.getParam("des_pos_x", initX);
    nh.getParam("des_pos_y", initY);
    
    nh.getParam("/num_robot", num_robot);
    //ROS_INFO("%d, init pos %d,%d", robot_ID, initX, initY);
    // Subscribers & Services Clients
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber map_sub = nh.subscribe("/node_map", 1000, mapCallback);
    ros::Subscriber laser_sub = nh.subscribe("front_laser/scan", 1000, laserCallback);
    // Publishers & Services Servers
    ros::Publisher visitedPub, desiredPub;
    visitedPub = nh.advertise<geometry_msgs::Point32>("/node_visited", 5);
    desiredPub = nh.advertise<geometry_msgs::Point32>("/node_desired", 5);

    // Loop variables
    double robotX, robotY, robotYaw, lastRobotX, lastRobotY, lastRobotYaw;
    int nearestNodeX, nearestNodeY;
    int lastNodeVisitedX = map_size * map_size;
    int lastNodeVisitedY = map_size * map_size;
    bool newNode = false;
    bool obstacleDetected = false;
    bool inMap = false;
    string direction = "right";
    geometry_msgs::Point32 node_visited;
    geometry_msgs::Point32 node_desired; //to publish the desired position in order to avoid collision between robots
    nh.getParam("des_pos_x", node_desired.x);
    nh.getParam("des_pos_y", node_desired.y);
    node_desired.z = robot_ID;
    desiredPub.publish(node_desired);
  
    ros::Rate rate(50);

    // Exploration phase
    while (ros::ok())
    {
        // Perform callbacks
        ros::spinOnce();
        // Store current position
        robotX = robotPose.position.x;
        robotY = robotPose.position.y;

        // Find nearest node
        nearestNodeX = round(robotX);
        nearestNodeY = round(robotY);

        if (inNode(robotX, robotY, nearestNodeX, nearestNodeY) && not inMap)
        {
            //ROS_INFO("Planner - Robot %d goal in %d,%d - Current position: %d,%d ", robot_ID, initX, initY, nearestNodeX, nearestNodeY);
            if (inInitPos(nh, nearestNodeX, nearestNodeY, initX, initY))
                inMap = true;
        }
        if (inMap)
        {
            try
            {
                int obstX, obstY; // Check if your desired position is actually an obstacle
                nh.getParam("des_pos_x", obstX);
                nh.getParam("des_pos_y", obstY);
                ROS_INFO("Planner - Robot %d obstacle in DP -> %d == %d ", robot_ID, node_map.at(obstX + LIMIT).at(obstY + LIMIT), -(num_robot + 1));

                if (node_map.at(obstX + LIMIT).at(obstY + LIMIT) == -(num_robot + 1))
                {
                    ROS_INFO("Planner - Robot %d obstacle detected", robot_ID);
                    nh.setParam("des_pos_x", lastNodeVisitedX);
                    nh.setParam("des_pos_y", lastNodeVisitedY);
                    obstacleDetected = true;
                    continue;
                }
            }
            catch (const std::exception &e)
            {
                ROS_INFO("Planner - Robot %d out of map", robot_ID);
            }

            // Check if we are in a new node
            if (inNode(robotX, robotY, nearestNodeX, nearestNodeY))
            {
                // ROS_INFO("Planner - Robot %d is in a node %d,%d", robot_ID, nearestNodeX, nearestNodeY);
                newNode = ((lastNodeVisitedX != nearestNodeX) || (lastNodeVisitedY != nearestNodeY));

                lastNodeVisitedX = nearestNodeX;
                lastNodeVisitedY = nearestNodeY;
            }

            //ROS_INFO("Planner - Robot %d is waiting for a new pos \n newNode is %d \n ObstDet is %d", robot_ID, int(newNode),int(obstacleDetected));

            // If we are in a new node in the map, publish the position of the node we just entered
            if (newNode || obstacleDetected)
            {

                if (obstacleDetected)
                {
                    updateDesPos(nh, nearestNodeX, nearestNodeY, robot_ID);
                    obstacleDetected = false;
                }
                else
                {
                    updateDesPos(nh, nearestNodeX, nearestNodeY, robot_ID);
                    node_visited.x = nearestNodeX;
                    node_visited.y = nearestNodeY;
                    node_visited.z = robot_ID;
                    visitedPub.publish(node_visited);

                    nh.getParam("des_pos_x", node_desired.x);
                    nh.getParam("des_pos_y", node_desired.y);
                    node_desired.z = robot_ID;
                    desiredPub.publish(node_desired);
                    //ROS_INFO("Planner - Robot %d  new pos achived %d,%d", robot_ID, int(node_desired.x), int(node_desired.y));
                }
            }

            rate.sleep();
        }
    }

    return 0;
}