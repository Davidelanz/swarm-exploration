#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <iostream>

ros::Publisher pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
   {
     if (msg->pose.pose.position.y < -3){
	geometry_msgs::Twist vel;
        vel.linear.x =0;
        pub.publish(vel);
     }
     else{
	geometry_msgs::Twist vel;
        vel.linear.x =2;
        pub.publish(vel);
     
   }
}



int main(int argc, char **argv)
{


 ros::init(argc, argv, "controller");
 ros::NodeHandle n;
 pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
 ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);
 ros::Rate loop_rate(10);
 ros::spin();
 return 0;
}
