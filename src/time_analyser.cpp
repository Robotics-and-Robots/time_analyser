#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"

#include <sstream>

ros::Time begin;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ros::Duration spent_time;
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  spent_time = ros::Time::now() - begin;
  // ROS_INFO("Time: [%d]", ros::Time::now().toNSec());
  ROS_INFO("Time spent: [%d]", spent_time.toNSec());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);
    begin = ros::Time::now();
    // ROS_INFO("Time: [%d]", begin.sec);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}