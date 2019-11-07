/**
\file    controller_node.cpp
\brief   Controls the robot for the node counting algorithm
\author  Davide Lanza
\date    06/10/2019
 * Parameters:
 * 
 * Subscribes to: 
 * 
 * Publishes to: 
 * 
*/

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic types you use.
#include "geometry_msgs/Twist.h"
#include "std_msgs/builtin_int16.h"
#include "std_msgs/builtin_int32.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
// ...

// You may have a number of globals here.
// ros::Publisher publisher;
// ...

// Callback storage variables:
geometry_msgs::Pose last_pose;

// Callback functions:
void poseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg){
   last_pose = pose_msg->pose.pose;
}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "controller");
   ROS_INFO("Controller node initialized\n");

   // Define your node handles
   ros::NodeHandle nh_;

   // Read the node parameters if any
   // ...

   // Declare your node's subscriptions and service clients
   ros::Subscriber odom_sub = nh_.subscribe("/odom", 1, poseCallback);

   // Declare your publishers and service servers
   static tf::TransformBroadcaster tf_pub;
   // ??? ADD command message publisher
 
   // Variables initialization
   std::string joint_names[] = {"joint_wheel1", "joint_wheel2", "joint_wheel3", "joint_wheel4"};
   tf::Transform tf_transform;
   // ???_msgs::JointCommand command_msg;


   ros::Rate rate(10);
   while(ros::ok()){

      ros::spinOnce();

      // Here we have to build "manually" the fields of the message to publish:
      // ????? to command_msg
      // then publish the command message
      
      // Publish tf transfrom
      tf_transform.setOrigin(
         tf::Vector3(last_pose.position.x,
                  last_pose.position.y,
                  last_pose.position.z) );
      tf_transform.setRotation(
         tf::Quaternion(-last_pose.orientation.x, 
                        last_pose.orientation.y, 
                        last_pose.orientation.z, 
                        last_pose.orientation.w));
      tf_pub.sendTransform(
         tf::StampedTransform(tf_transform, ros::Time::now(), "base_link", "odom"));

      rate.sleep();
   }

   return 0;

}
