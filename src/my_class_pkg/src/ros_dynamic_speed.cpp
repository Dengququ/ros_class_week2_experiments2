#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <my_class_pkg/TutorialsConfig.h>

namespace {

double robot_speed = 0.0;

void callback(my_class_pkg::TutorialsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
           config.int_param,
           config.double_param,
           config.str_param.c_str(),
           config.bool_param ? "True" : "False",
           config.size);
  robot_speed = config.double_param;
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_dynamic_speed_node");

  ros::NodeHandle nh;
  dynamic_reconfigure::Server<my_class_pkg::TutorialsConfig> server;
  dynamic_reconfigure::Server<my_class_pkg::TutorialsConfig>::CallbackType f;

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Dynamic speed node started, publishing to /cmd_vel");

  ros::Rate rate(10);
  while (ros::ok()) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = robot_speed;
    cmd_pub.publish(cmd_vel);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
