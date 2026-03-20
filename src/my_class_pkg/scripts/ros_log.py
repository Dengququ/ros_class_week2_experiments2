#!/usr/bin/env python3

import rospy


if __name__ == '__main__':
    rospy.init_node('ros_logging_example')

    rospy.logdebug('This is a DEBUG message.')
    rospy.loginfo('This is an INFO message.')
    rospy.logwarn('This is a WARNING message.')
    rospy.logerr('This is an ERROR message.')
    rospy.logfatal('This is a FATAL message.')

    rospy.spin()
