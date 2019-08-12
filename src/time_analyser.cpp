/* @brief: count spent time to send and receive message. publish rate can be dinamically defined running on temrinal: rosparam set /time_analyser_rate 100.0*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"

#include <sstream>

ros::Time begin;
int counter = 0;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ros::Duration spent_time;
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  spent_time = ros::Time::now() - begin;
  // ROS_INFO("Time: [%d]", ros::Time::now().toNSec());
  ROS_INFO("Time spent: %d: [%d]", counter, spent_time.toNSec());
  ++counter;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  double rate;
  n.param("/time_analyser_rate", rate, 2.0);

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(rate);

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    chatter_pub.publish(msg);

    ROS_INFO("%s", msg.data.c_str());

    begin = ros::Time::now();
    // ROS_INFO("Time: [%d]", begin.sec);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    n.getParam("/time_analyser_rate", rate);
    loop_rate = ros::Rate(rate);
  }

  return 0;
}