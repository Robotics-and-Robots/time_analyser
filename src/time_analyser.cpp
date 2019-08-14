/* @brief: count spent time to send and receive message. publish rate can be dinamically defined running on temrinal: rosparam set /time_analyser_rate 100.0 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"
#include <unordered_map> 

#include <sstream>
#include <fstream>

std::unordered_map<std::string, ros::Time> umap; 

ros::Time begin;
int counter = 0;
ros::Duration spent_time;

std::ofstream myfile("/home/adomingues/gaph/darlan/dataCollection_1byte.txt", std::ios::out | std::ios::binary);

void mpsocToRosCallback(const std_msgs::String::ConstPtr& msg)
{
  spent_time = ros::Time::now() - begin;
  // ROS_INFO("Spent time: %d: [%d nano seconds]", counter, spent_time.toNSec());

  std::unordered_map<std::string, ros::Time>::const_iterator got = umap.find(msg->data);
  std::cout << "Recebeu: "<< got->first << " ---- demorou: " << spent_time.toNSec() << "nsecs" << std::endl;

  myfile << got->first << "," << spent_time.toNSec() << std::endl;
  
  counter++;
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

  uint8_t count = 0;
  while (ros::ok())
  {
    // msg to be written
    std_msgs::String  msg;
    std::stringstream ss;

    // msg size control
    for (int i = 0; i < 1; i++)
    {
      ss << std::to_string(count);
    }

    msg.data = ss.str();
    // ROS_INFO("data: %d, data: %s, sizeof: %d", count, msg.data.c_str(), msg.data.size());

    orca_ros_to_mpsoc_pub.publish(msg);

    begin = ros::Time::now();
    umap[msg.data] = begin;

    ros::spinOnce();

    loop_rate.sleep();
    count++;

    n.getParam("/time_analyser_rate", rate);
    loop_rate = ros::Rate(rate);

    // if (count == 30) {
    //   for (auto x : umap) 
    //     std::cout << x.first.c_str() << " " << x.second << std::endl;
    //   std::unordered_map<std::string, ros::Time>::const_iterator got = umap.find(std::to_string(count - 1));
    //   std::cout << "Publicou: "<< got->first << " ---- no tempo: " << got->second << std::endl;
    // }
      
  }

  myfile.close();
  return 0;
}