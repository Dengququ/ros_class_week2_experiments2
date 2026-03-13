#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

// 安全距离阈值（米）
static const float SAFE_DIST = 0.4f;

ros::Publisher cmd_pub;

float dist_front = 9.9f;

void cb_front(const sensor_msgs::Range::ConstPtr& msg) { dist_front = msg->range; }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tof_avoid_node");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber s2 = nh.subscribe<sensor_msgs::Range>("/ul/sensor2", 10, cb_front);

    ros::Rate rate(10);

    ROS_INFO("TOF avoidance node started.");

    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Twist cmd;

        if (dist_front < SAFE_DIST)
        {
            // 前方有障碍：后退
            cmd.linear.x  = -0.15;
            cmd.angular.z = 0.0;
            ROS_WARN("Obstacle front (%.2f m)! Backing up.", dist_front);
        }
        else
        {
            // 无障碍：前进
            cmd.linear.x  = 0.2;
            cmd.angular.z = 0.0;
        }

        cmd_pub.publish(cmd);
        rate.sleep();
    }

    return 0;
}
