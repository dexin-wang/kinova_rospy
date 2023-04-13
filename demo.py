#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import roslib
from kinova.kinova_control import *

if __name__ == '__main__':
    try:
        # ROS节点初始化
        rospy.init_node('kinova_test')
        
        rospy.loginfo('init kinova')
        kinova_robotTypeParser()
        getcurrentCartesianCommand()

        rospy.loginfo('move arm')
        move_arm('mdeg', [0.2, -0.45, 0.2, 180, 0, 0], False)
        rospy.loginfo('move gripper')
        move_finger('percent', [45, 45, 45])

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
