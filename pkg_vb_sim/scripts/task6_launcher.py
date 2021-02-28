#!/usr/bin/env python

import rospy
import rospkg
import subprocess

if __name__ == '__main__':
    rospy.init_node('task6_launcher')

    rp = rospkg.RosPack()
    str_pkg_path = rp.get_path('pkg_vb_sim')
    rospy.loginfo( "Package Path: {}".format(str_pkg_path) )

    res = subprocess.call(["python2", "{}/src/task6_random_config_generator.exe".format(str_pkg_path)])

    rospy.spin()