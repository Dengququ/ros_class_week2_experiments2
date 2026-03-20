#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_logging_example");
  ros::NodeHandle nh;

  ROS_DEBUG("This is a DEBUG message.");
  ROS_INFO("This is an INFO message.");
  ROS_WARN("This is a WARNING message.");
  ROS_ERROR("This is an ERROR message.");
  ROS_FATAL("This is a FATAL message.");

  ros::spin();
  return 0;
}
