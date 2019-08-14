/* @brief: count spent time to send and receive message. publish rate can be dinamically defined running on temrinal: rosparam set /time_analyser_rate 100.0 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"
#include <unordered_map> 
#include <string>

#include <sstream>
#include <fstream>

#define NUM_MSGS 100

// Hash to store msg and publishing time
std::unordered_map<std::string, ros::Time> umap; 

// Time measurement
ros::Time     begin;
ros::Duration spent_time;

// File writing
std::ofstream myfile("/home/adomingues/gaph/darlan/dataCollection_1byte.txt", std::ios::out | std::ios::binary);

bool flag_100msgs_read = false;

// Callback executed when mpsoc publish into /mpsoc_to_ros topic
// Here we measure the travel time of the sent message
void mpsocToRosCallback(const std_msgs::String::ConstPtr& msg)
{
  spent_time = ros::Time::now() - begin;
  std::unordered_map<std::string, ros::Time>::const_iterator got = umap.find(msg->data);
  myfile << got->first << ", " << spent_time.toNSec() << std::endl;

  if (msg->data.compare("99") == 0)
  {
    flag_100msgs_read = true;
  }
}

int main(int argc, char **argv)
{

  // ROS node handler and initializer
  ros::init(argc, argv, "time_analyser");
  ros::NodeHandle n;

  // Parametrizable publishing rate
  double rate;
  n.param("/time_analyser_rate", rate, 2.0);

  // test message publisher 
  ros::Publisher orca_ros_to_mpsoc_pub = n.advertise<std_msgs::String>("orca_ros_to_mpsoc", 1000);
  ros::Rate loop_rate(rate);

  // mpsoc publishing listener
  ros::Subscriber orca_mpsoc_to_ros_pub = n.subscribe("orca_mpsoc_to_ros", 1000, mpsocToRosCallback);

  uint8_t count = 0;
  while (ros::ok() && (!flag_100msgs_read))
  {
    // msg to be written
    std_msgs::String  msg;
    std::stringstream ss;

    // msg's size control
    for (int i = 0; i < 1; i++)
    {
      ss << std::to_string(count);
    }

    msg.data = ss.str();
    orca_ros_to_mpsoc_pub.publish(msg);

    // publishing time 
    begin = ros::Time::now();
    umap[msg.data] = begin;

    ros::spinOnce();

    loop_rate.sleep();
    count++;

    n.getParam("/time_analyser_rate", rate);
    loop_rate = ros::Rate(rate);
      
  }

  myfile.close();
  return 0;
}