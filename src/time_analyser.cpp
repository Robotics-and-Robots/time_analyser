/* @brief: count spent time to send and receive message. publish rate can be dinamically defined running on temrinal: rosparam set /time_analyser_rate 100.0 */
#include "ros/ros.h"			// Basic ROS lib
#include "std_msgs/String.h"	// Main publishing message
//#include "ros/time.h"			// Time measurement
#include <unordered_map> 		// Msg and time store
#include <string>				// Main message type
#include <sstream>				// Message assembling
#include <fstream>				// Measurement output
#include <time.h>       		// Time measurement
#include <chrono>				// Time measurement
#include <cmath>				// Time measurement

#define NUM_MSGS 100			// Sampling amount

// Hash to store msg and publishing time
//std::unordered_map<std::string, ros::Time> umap; 
std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> umap; 
std::chrono::high_resolution_clock::time_point begin, end;

// Time measurement
//ros::Time     begin;
//ros::Duration spent_time;
bool 	flag_100msgs_read	= false;
int  	num_msgs_counter    = 0;

// File writing
std::ofstream myfile("/home/adomingues/gaph/darlan/ursa/tools/ros-integration/src/time_analyser/dataCollection_1024B_tile1-0.txt", std::ios::out | std::ios::binary);

// Callback executed when mpsoc publish into /mpsoc_to_ros topic
// Here we measure the travel time of the sent message
void mpsocToRosCallback(const std_msgs::String::ConstPtr& msg)
{
  

  end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>( end - begin ).count();

  std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point>::const_iterator got = umap.find(msg->data);
  if(got != umap.end())
  {
  
//    myfile << got->first << ", " << spent_time.toNSec() << std::endl; // ROS Version
    myfile << got->first << ", " << duration << std::endl; // Chrono Version
    umap.erase(msg->data);

    if(num_msgs_counter == NUM_MSGS)
    {
      flag_100msgs_read = true;
    }

    num_msgs_counter++;

    if (umap.size() > 0)
    {
      umap.erase(umap.begin(), umap.end());
    }
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

  // ROS message publisher 
  ros::Publisher orca_ros_to_mpsoc_pub = n.advertise<std_msgs::String>("orca_ros_to_mpsoc", 1000);
  ros::Rate loop_rate(rate);

  // mpsoc publishing listener
  ros::Subscriber orca_mpsoc_to_ros_pub = n.subscribe("orca_mpsoc_to_ros", 1000, mpsocToRosCallback);

  srand (time(NULL));

  while (ros::ok() && (!flag_100msgs_read))
  {
    // msg to be written
    std_msgs::String  msg;
    std::stringstream ss;

    // msg's size control
    for (int i = 0; i < 1024; i++)
    {
      ss << std::to_string(std::rand() % 10);
    }

    msg.data = ss.str();
    ROS_INFO("String: %s, string size: %d", msg.data.c_str(), msg.data.size());

    orca_ros_to_mpsoc_pub.publish(msg);
    begin = std::chrono::high_resolution_clock::now();

    ROS_INFO("String: %s, string size: %d", msg.data.c_str(), msg.data.size());

    // collects publishing time 
    //begin = ros::Time::now();
    umap[msg.data] = begin;
    
    ros::spinOnce();
    loop_rate.sleep();
    n.getParam("/time_analyser_rate", rate);
    loop_rate = ros::Rate(rate);
    
  }

  myfile.close();
  return 0;
}

