#!/usr/bin/env python  
import rospy

import tf2_ros
import tf2_msgs.msg


class tfEcho:

    def __init__(self):
        rospy.init_node('node_tf_echo')
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)


    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            rospy.loginfo(  "\n" +
                            "Translation: \n" +
                            "x: {} \n".format(trans.transform.translation.x) +
                            "y: {} \n".format(trans.transform.translation.y) +
                            "z: {} \n".format(trans.transform.translation.z) +
                            "\n" +
                            "Orientation: \n" +
                            "x: {} \n".format(trans.transform.rotation.x) +
                            "y: {} \n".format(trans.transform.rotation.y) +
                            "z: {} \n".format(trans.transform.rotation.z) +
                            "w: {} \n".format(trans.transform.rotation.w) )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")



def main():
    my_tf = tfEcho()

    reference_frame = "world"
    target_frame = "ur5_wrist_3_link"

    while not rospy.is_shutdown():
        my_tf.func_tf_print(reference_frame, target_frame)
        rospy.sleep(1)

    del my_tf


if __name__ == '__main__':
    main()