#include <ros/ros.h>
#include <learning_actionlib/FibonacciAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "fibonaccicancel");

  actionlib::SimpleActionClient<learning_actionlib::FibonacciAction> ac("fibonacci", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the fibonacci server to come up");
  }

  ROS_INFO("Cancelling goal");
  ac.cancelAllGoals();

  return 0;
}

