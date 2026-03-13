#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"

ros::Publisher cmd_pub;

// 碰撞传感器编号：0=左前方  1=正前方  2=右前方
void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    geometry_msgs::Twist cmd;

    bool left    = (msg->data.size() > 0 && msg->data[0]);
    bool front   = (msg->data.size() > 1 && msg->data[1]);
    bool right_s = (msg->data.size() > 2 && msg->data[2]);

    if (front || left || right_s)
    {
        // 有碰撞：先停止
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub.publish(cmd);
        ros::Duration(0.3).sleep();

        // 后退
        cmd.linear.x  = -0.15;
        cmd.angular.z = 0.0;
        cmd_pub.publish(cmd);
        ros::Duration(1.0).sleep();

        // 根据碰撞方向决定转向：左/正前方 → 右转；右前方 → 左转
        cmd.linear.x  = 0.0;
        if (right_s && !left)
            cmd.angular.z =  0.5;   // 左转
        else
            cmd.angular.z = -0.5;   // 右转
        cmd_pub.publish(cmd);
        ros::Duration(1.5).sleep();

        // 转向结束，停止等待下一帧判断
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub.publish(cmd);

        if (front)
            ROS_WARN("Front bumped! Backing up and turning.");
        else if (left)
            ROS_WARN("Left bumped! Backing up and turning right.");
        else
            ROS_WARN("Right bumped! Backing up and turning left.");
    }
    else
    {
        // 无碰撞：正常前进
        cmd.linear.x  = 0.2;
        cmd.angular.z = 0.0;
        cmd_pub.publish(cmd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bump_avoid_node");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("/robot/bump_sensor", 10, bumpCallback);

    ROS_INFO("Bump avoidance node started. Moving forward...");
    ros::spin();
    return 0;
}
