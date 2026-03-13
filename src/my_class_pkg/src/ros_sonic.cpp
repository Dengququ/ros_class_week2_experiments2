#include <ros/ros.h>
#include <sensor_msgs/Range.h>

void rangeCallback1(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("Distance Left:  %f m", msg->range);
}

void rangeCallback2(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("Distance Front: %f m", msg->range);
}

void rangeCallback3(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("Distance Right: %f m", msg->range);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::Range>("/ul/senser1", 10, rangeCallback1);
    ros::Subscriber sub_2 = nh.subscribe<sensor_msgs::Range>("/ul/senser2", 10, rangeCallback2);
    ros::Subscriber sub_3 = nh.subscribe<sensor_msgs::Range>("/ul/senser3", 10, rangeCallback3);

    ros::spin();
    return 0;
}
