#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

// 安全距离阈值（米）
static const float SAFE_DIST = 0.3f;

ros::Publisher cmd_pub;

// 各方向最新距离，初始化为大值（未收到数据视为安全）
float dist_left  = 9.9f;
float dist_front = 9.9f;
float dist_right = 9.9f;

void cb_left(const sensor_msgs::Range::ConstPtr& msg)  { dist_left  = msg->range; }
void cb_front(const sensor_msgs::Range::ConstPtr& msg) { dist_front = msg->range; }
void cb_right(const sensor_msgs::Range::ConstPtr& msg) { dist_right = msg->range; }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tof_avoid_node");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 订阅 TOF 三个方向
    ros::Subscriber s1 = nh.subscribe<sensor_msgs::Range>("/us/tof1", 10, cb_left);
    ros::Subscriber s2 = nh.subscribe<sensor_msgs::Range>("/us/tof2", 10, cb_front);
    ros::Subscriber s3 = nh.subscribe<sensor_msgs::Range>("/us/tof3", 10, cb_right);

    ros::Rate rate(10);  // 10 Hz 控制循环

    ROS_INFO("TOF avoidance node started.");

    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Twist cmd;

        if (dist_front < SAFE_DIST)
        {
            // 正前方有障碍：停止并转向
            cmd.linear.x  = 0.0;

            if (dist_left > dist_right)
                cmd.angular.z =  0.5;   // 左转（左侧更空旷）
            else
                cmd.angular.z = -0.5;   // 右转

            ROS_WARN("Obstacle front (%.2f m)! Turning.", dist_front);
        }
        else if (dist_left < SAFE_DIST)
        {
            // 左侧过近：右偏
            cmd.linear.x  = 0.15;
            cmd.angular.z = -0.3;
            ROS_WARN("Obstacle left (%.2f m)! Turning right.", dist_left);
        }
        else if (dist_right < SAFE_DIST)
        {
            // 右侧过近：左偏
            cmd.linear.x  = 0.15;
            cmd.angular.z =  0.3;
            ROS_WARN("Obstacle right (%.2f m)! Turning left.", dist_right);
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
