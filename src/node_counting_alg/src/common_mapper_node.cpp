#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
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
ros::Publisher mapPub;
int curr_node_x, curr_node_y, robot_node, curr_obst_x, curr_obst_y, robot_obst;

// "vector<int>(map_size,0));" create a vector of size "map_size" with all values as 0.
// we create a vector of those vectors
int map_size = 2 * LIMIT + 1;
vector<vector<int>> node_map(map_size, vector<int>(map_size, 0));

// Callbacks

void currVisitedCallback(const geometry_msgs::Point32 &msg)
{
    // (x, y, robot_ID)
    curr_node_x = msg.x;
    curr_node_y = msg.y;
    robot_node = msg.z;
}

void obstacleCallback(const geometry_msgs::Point &msg)
{
    // (x, y, robot_ID)
    //curr_obst_x = static_cast<int>(msg.x);
    curr_obst_x = msg.x;
    curr_obst_y = msg.y;
    robot_obst = msg.z;
}

// Map utils

string mapCoder()
{
    // ROS_INFO("I'm starting to code the map");
    string s;
    for (int y = 0; y < map_size; y++)
        for (int x = 0; x < map_size; x++)
        {
            s.append(to_string(node_map.at(x).at(y)));
            s.append(",");
        }

    // ROS_INFO("Finished to code the map");
    return s;
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

void printMap()
{
    for (int y = map_size - 1; y >= 0; y--)
    {
        for (int x = 0; x < map_size; x++)
        {
            if (node_map.at(x).at(y) == -1)
                std::cout << "- ";
            else if (node_map.at(x).at(y) == 0)
                std::cout << "? ";
            else
                std::cout << node_map.at(x).at(y) << " ";
        }
        std::cout << endl;
    }
    std::cout << endl;
}

// Map updaters

void mapUpdate(int index, int curr_node_x, int curr_node_y,
               int last_node_x, int last_node_y, bool obstacle = false)
{
    try
    {
        if (!obstacle)
            node_map.at(curr_node_x + LIMIT).at(curr_node_y + LIMIT)++;
        else
        {
            if (node_map.at(curr_node_x + LIMIT).at(curr_node_y + LIMIT) == 0)
                node_map.at(curr_node_x + LIMIT).at(curr_node_y + LIMIT) = -1;
        }
        // If you can update the map, then print it in sceen and publish it
        std_msgs::String codedMap;
        codedMap.data = mapCoder();
        printMap();
        mapPub.publish(codedMap);
    }
    catch (const std::exception &e)
    {
        if (!obstacle)
            ROS_INFO("Robot %d out of bounds \n", index + 1);
        else
            ROS_INFO("Obstacle in front of Robot %d is out of bounds \n", index + 1);
    }
}

// Main
int main(int argc, char **argv)
{

    // Node Initialization
    ros::init(argc, argv, "mapper");
    ros::NodeHandle nh;

    int num_robot;
    nh.getParam("num_robot", num_robot);

    // Subscribers & Services Clients
    ros::Subscriber sub = nh.subscribe("/node_visited", 100, currVisitedCallback);
    ros::Subscriber obst_sub = nh.subscribe("/obstacle_node", 100, obstacleCallback);

    // Publishers & Services Servers
    mapPub = nh.advertise<std_msgs::String>("node_map", 5);

    // Loop variables;
    int last_node_x, last_node_y, last_obst_x, last_obst_y;
    vector<vector<int>> last_node(num_robot, vector<int>(2, map_size));
    curr_node_x = map_size;
    curr_node_y = map_size;
    robot_node = 1;
    vector<vector<int>> last_obst(num_robot, vector<int>(2, map_size));
    curr_obst_x = map_size;
    curr_obst_y = map_size;
    robot_obst = 1;

    ros::Rate rate(50);

    /*while ((curr_node_x > LIMIT || curr_node_y > LIMIT))
        // Wait to start the actual process until
        // a valid position from some robot has been sent
        ros::spinOnce();
        */

    while (ros::ok())
    {
        // Perform callbacks
        ros::spinOnce();

        // Get node data
        try
        {
            last_node_x = last_node.at(robot_node - 1).at(0);
            last_node_y = last_node.at(robot_node - 1).at(1);

            // Update map count
            bool new_node = (curr_node_x != last_node_x || curr_node_y != last_node_y);
            if (new_node)
            {
                ROS_INFO("Robot %d in node (%d, %d) \n", robot_node, curr_node_x, curr_node_y);
                mapUpdate(robot_node - 1, curr_node_x, curr_node_y, last_node_x, last_node_y);
                try
                {
                    // Save this new node for the robot
                    last_node.at(robot_node - 1).at(0) = curr_node_x;
                    last_node.at(robot_node - 1).at(1) = curr_node_y;
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Storing current_node info from last_node array failed" << '\n';
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Extracting info from last_node array failed"
                      << '\n'
                      << e.what() << '\n';
        }

        // Get obstacle data
        try
        {
            last_obst_x = last_obst.at(robot_obst - 1).at(0);
            last_obst_y = last_obst.at(robot_obst - 1).at(1);

            bool new_obst = (curr_obst_x != last_obst_x || curr_obst_y != last_obst_y);
            if (new_obst)
            {
                ROS_INFO("Obstacle found in node (%d, %d) \n", curr_obst_x, curr_obst_y);
                // Update obstacle (last arg = "true") count
                mapUpdate(robot_obst, curr_obst_x, curr_obst_y, last_obst_x, last_obst_y, true);
                try
                {
                    // Save this new node for the robot
                    last_obst.at(robot_obst - 1).at(0) = curr_obst_x;
                    last_obst.at(robot_obst - 1).at(1) = curr_obst_y;
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Storing current_obstacle info from last_node array failed" << '\n';
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Extracting info from last_obstacle array failed"
                      << '\n'
                      << e.what() << '\n';
        }

        rate.sleep();
    }

    return 0;
}
