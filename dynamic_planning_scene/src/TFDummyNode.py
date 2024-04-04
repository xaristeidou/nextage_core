#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import random

TF_NAMES = ['big_hob_frame','small_hob_frame','toolbox_frame','double_hob_frame']

def publish_tf():
    rospy.init_node('publish_tf_node')
    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(0.5)  # 0.1 Hz, i.e., every 10 seconds

    while not rospy.is_shutdown():
        for i in range(4):
            tf = geometry_msgs.msg.TransformStamped()
            tf.header.stamp = rospy.Time.now()
            tf.header.frame_id = "world"
            tf.child_frame_id = TF_NAMES[i]

            # Generate random position
            tf.transform.translation.x = random.uniform(0, 2)
            tf.transform.translation.y = random.uniform(0, 2)
            tf.transform.translation.z = random.uniform(0, 3)

            # Set orientation
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = 0.0
            tf.transform.rotation.w = 1.0

            broadcaster.sendTransform(tf)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_tf()
    except rospy.ROSInterruptException:
        pass
