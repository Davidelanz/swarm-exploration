#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
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
geometry_msgs::Point32 curr_node;

// "vector<int>(map_size,0));" create a vector of size "map_size" with all values as 0.
// we create a vector of those vectors
int map_size = 2 * LIMIT + 1;
vector<vector<int>> node_map(map_size, vector<int>(map_size, 0));

// Callbacks
void currVisitedCallback(const geometry_msgs::Point32 &msg)
{
    curr_node.x = msg.x; // x
    curr_node.y = msg.y; // y
    curr_node.z = msg.z; // robot_ID
}

string mapCoder()
{
    // ROS_INFO("I'm starting to code the map");
    string s;
    for (int y = 0; y < map_size; y++)
    {
        for (int x = 0; x < map_size; x++)
        {
            s.append(to_string(node_map.at(x).at(y)));
            s.append(",");
        }
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
        int x = i % map_size;
        int y = i / map_size;
        decodedMap.at(x).at(y) = stoi(token);
        s.erase(0, pos + delimiter.length());
        i++;
    }

    // ROS_INFO("Finished to decode the map");
    return decodedMap;
}

void printMap(std_msgs::String codedMap)
{
    for (int y = map_size - 1; y >= 0; y--)
    {
        for (int x = 0; x < map_size; x++)
        {
            cout << mapDecoder(codedMap.data).at(x).at(y) << " ";
        }
        cout << endl;
    }
    cout << endl;
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
    ros::Subscriber sub = nh.subscribe("/node_visited", 1000, currVisitedCallback);

    // Publishers & Services Servers
    ros::Publisher mapPub;
    mapPub = nh.advertise<std_msgs::String>("node_map", 5);

    // Loop variables;
    vector<vector<int>> last_node(num_robot, vector<int>(2, map_size));
    curr_node.x = map_size;
    curr_node.y = map_size;
    curr_node.z = 0;
    std_msgs::String codedMap;
    int last_x, last_y, robot;

    ros::Rate rate(10);
    while (ros::ok())
    {
        // Perform callbacks
        ros::spinOnce();

        // Get data
        robot = curr_node.z - 1;
        try
        {
            // index starts from 0, robot ID from 1
            last_x = last_node.at(robot).at(0);
            last_y = last_node.at(robot).at(1);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Extracting info from last node array failed"
                      << '\n'
                      << e.what() << '\n';
        }

        // Update map count
        bool new_node = (curr_node.x != last_x || curr_node.y != last_y);
        if (new_node)
        {
            try
            {
                node_map.at(curr_node.x + LIMIT).at(curr_node.y + LIMIT)++;

                // If you can update the map, then print it in sceen and publish it
                codedMap.data = mapCoder();
                printMap(codedMap);
                mapPub.publish(codedMap);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Failed to update map count"
                          << '\n'
                          << e.what() << '\n';
            }
            try
            {
                // Save this new node for the robot
                last_node.at(robot).at(0) = curr_node.x;
                last_node.at(robot).at(1) = curr_node.y;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Storing current nodeinfo from last node array failed"
                          << '\n'
                          << e.what() << '\n';
            }
            
        }

        rate.sleep();
    }

    return 0;
}
