#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess as sub
from std_msgs.msg import Bool
from std_msgs.msg import String

class NodeController:
    def __init__(self):
        self._string_sub = rospy.Subscriber("/localization_switch_name", String, self.callback)
        self._p1 = sub.Popen(['roslaunch', 'hnd_launch', 'amcl.launch'])
        
    def callback(self, msg):
        if msg.data == "gmapping":
            if self._p1:
                self._p1.terminate()
                self._p1 = sub.Popen(['roslaunch', 'tsukuba_challenge2019', 'amcl_to_gmapping.launch'])
        elif msg.data == "amcl":
            if self._p1:
                self._p1.terminate()
                self._p1 = sub.Popen(['roslaunch', 'tsukuba_challenge2019', 'gmapping_to_amcl.launch'])
        print(msg.data)


if __name__ == '__main__':
    rospy.init_node('localization_switcher')
    nc = NodeController()
    rospy.spin()