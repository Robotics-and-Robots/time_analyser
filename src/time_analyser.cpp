/* @brief: count spent time to send and receive message. publish rate can be dinamically defined running on temrinal: rosparam set /time_analyser_rate 100.0 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"

#include <sstream>

#include <fstream>

ros::Time begin;
int counter = 0;

std::ofstream myfile("/home/darlan/Desktop/example.txt", std::ios::out | std::ios::binary);

void mpsocToRosCallback(const std_msgs::String::ConstPtr& msg)
{
  ros::Duration spent_time;
  spent_time = ros::Time::now() - begin;
  ROS_INFO("Spent time: %d: [%d nano seconds]", counter, spent_time.toNSec());

  myfile << spent_time.toNSec();

  // spent_times[msg.str_c().atoi()] = spent_time.toNSec();
  // if (msg.str_c().atoi() == 1000)
  //   myfile << spent_times;
  
  ++counter;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "time_analyser");
  ros::NodeHandle n;

  double rate;
  n.param("/time_analyser_rate", rate, 2.0);

  ros::Publisher orca_ros_to_mpsoc_pub = n.advertise<std_msgs::String>("orca_ros_to_mpsoc", 1000);
  ros::Rate loop_rate(rate);

  ros::Subscriber orca_mpsoc_to_ros_pub = n.subscribe("orca_mpsoc_to_ros", 1000, mpsocToRosCallback);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    orca_ros_to_mpsoc_pub.publish(msg);

    begin = ros::Time::now();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    n.getParam("/time_analyser_rate", rate);
    loop_rate = ros::Rate(rate);
  }

  myfile.close();
  return 0;
}