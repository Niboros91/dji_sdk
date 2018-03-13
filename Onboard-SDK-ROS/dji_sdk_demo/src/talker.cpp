#include "ros/ros.h"
#include "dji_sdk/test.h" //To include the msg created

#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

 
  ros::NodeHandle n;

  
  ros::Publisher chatter_pub = n.advertise<dji_sdk::test>("testing", 1000);

  ros::Rate loop_rate(10);

  
  int count = 0;
  while (ros::ok())
  {
    
    dji_sdk::test msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

   
	
     msg.state=3;
     msg.point_reached=1;
    
    //ROS_INFO("%d", msg.state);
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

