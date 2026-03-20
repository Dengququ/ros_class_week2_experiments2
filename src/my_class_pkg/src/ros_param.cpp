#include <ros/ros.h>

#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_param_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string param_name = "my_param";
  std::string param_value = "hello";

  std::string my_param_first;
  private_nh.getParam(param_name, my_param_first);
  ROS_INFO("Private 1 The value of %s is %s", param_name.c_str(), my_param_first.c_str());

  private_nh.setParam(param_name, param_value);
  private_nh.getParam(param_name, my_param_first);
  ROS_INFO("Private 2 The value of %s is %s", param_name.c_str(), my_param_first.c_str());

  nh.setParam(param_name, param_value);

  std::string my_param;
  nh.getParam(param_name, my_param);
  ROS_INFO("The value of %s is %s", param_name.c_str(), my_param.c_str());

  ros::spin();

  return 0;
}
