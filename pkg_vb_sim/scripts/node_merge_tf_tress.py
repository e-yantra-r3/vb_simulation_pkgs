#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()



if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_tree_merge')

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    rospy.loginfo("TF Broadcaster Started.")

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "world"
    # t.child_frame_id = "ur5_2_tf/world"
    # t.transform.translation.x = 0.0
    # t.transform.translation.y = 0.0
    # t.transform.translation.z = 0.0
    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    # t.transform.rotation.x = q[0]
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]

    # br.sendTransform(t)

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "world"
    # t.child_frame_id = "ur5_1_tf/world"
    # t.transform.translation.x = 0.0
    # t.transform.translation.y = 7.0
    # t.transform.translation.z = 0.0
    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    # t.transform.rotation.x = q[0]
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]

    # br.sendTransform(t)

    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "ur5_2_tf/world"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "ur5_1_tf/world"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 7.0
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

        # t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "world"
        # t.child_frame_id = "ur5_2_frame"
        # t.transform.translation.x = 0.0
        # t.transform.translation.y = 0.0
        # t.transform.translation.z = 0.0
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # br.sendTransform(t)

        # t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "world"
        # t.child_frame_id = "ur5_1_frame"
        # t.transform.translation.x = 0.0
        # t.transform.translation.y = 7.0
        # t.transform.translation.z = 0.0
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # br.sendTransform(t)


    rospy.spin()