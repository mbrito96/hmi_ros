#!/usr/bin/env python

import rospy
import hmi_ros

if __name__ == '__main__':
    try:
        hmi_ros.hmi_start()
    except rospy.ROSInterruptException:
        pass