#include "dji_sdk/test.h" //To include the msg created
#include "ros/ros.h"



void testCallback(const dji_sdk::test& msg)
{
  ROS_INFO("I heard: [%d]", msg.state);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listen");

  
  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("testing", 1000, testCallback);

  
  ros::spin();

  return 0;
}
