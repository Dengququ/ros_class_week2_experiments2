#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

ros::Publisher cmd_pub;

// 状态标志
bool   got_initial = false;     // 是否已记录初始偏航角
bool   spin_done   = false;     // 是否完成旋转
double initial_yaw = 0.0;       // 初始偏航角（弧度）

// 从四元数提取偏航角（绕 Z 轴旋转角，右手坐标系）
double getYaw(const geometry_msgs::Quaternion& q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 将角度差规范化到 (-π, π]
double normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (spin_done) return;

    double current_yaw = getYaw(msg->orientation);

    // 第一帧：记录初始偏航角
    if (!got_initial)
    {
        initial_yaw = current_yaw;
        got_initial = true;
        ROS_INFO("Initial yaw: %.2f deg", initial_yaw * 180.0 / M_PI);
        return;
    }

    // 计算已旋转角度（绝对值）
    double turned = std::fabs(normalizeAngle(current_yaw - initial_yaw));
    ROS_INFO_THROTTLE(0.5, "Turned: %.1f / 180.0 deg", turned * 180.0 / M_PI);

    geometry_msgs::Twist cmd;

    if (turned < M_PI - 0.05)   // 还未到 180°（留 3° 余量）
    {
        // 持续左转
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.6;    // rad/s，可根据实际调整
    }
    else
    {
        // 达到 180°：停止
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        spin_done = true;
        ROS_INFO("Spin complete! Final yaw: %.2f deg", current_yaw * 180.0 / M_PI);
    }

    cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_spin_node");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, imuCallback);

    ROS_INFO("IMU spin node started. Will rotate 180 degrees.");
    ros::spin();

    return 0;
}
