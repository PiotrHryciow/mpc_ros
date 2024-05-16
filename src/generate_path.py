#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from tf.msg import tfMessage


def generate_path():
    rospy.init_node("path_gen", anonymous=True)
    pub_path = rospy.Publisher("/set_path", PoseArray, queue_size=10)
    sub_tf = rospy.Subscriber("/tf", tfMessage, tf_callback)
    path = PoseArray()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        # rate.sleep()


def tf_callback(transform: tfMessage):
    for tf in transform.transforms:
        if 1:
            translation = tf.transform.translation


if __name__ == "__main__":
    try:
        generate_path()
    except rospy.ROSInterruptException:
        pass
