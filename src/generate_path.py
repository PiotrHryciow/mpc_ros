#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import PoseArray, Vector3, Pose

# from tf.msg import tfMessage
import numpy as np

# import tf2_ros
from tf2_msgs.msg import TFMessage

import matplotlib.pyplot as plt

translation = Vector3()


def generate_path():
    rospy.init_node("path_gen", anonymous=True)

    pub_path = rospy.Publisher("/set_path", PoseArray, queue_size=10)
    sub_tf = rospy.Subscriber("/tf", TFMessage, tf_callback)
    point_list_x = np.array([1, 3, 5, -8])
    point_list_y = np.array([-2, -1, 3, 2])
    point_nr = 20
    path = PoseArray()
    path.header.frame_id = "odom"
    rate = rospy.Rate(10)  # 10hz
    # coefs = np.polyfit(point_list_x[0:1], point_list_y[0:1], 5)
    # xp = np.linspace(-1, 1, 100)
    # p = np.poly1d(coefs)
    # plt.plot(xp, p(xp))
    # plt.show()

    input("Press Enter to send PoseArray")

    pose_list = []
    for i in range(point_list_x.size - 1):
        x = np.linspace(point_list_x[i], point_list_x[i + 1], num=point_nr)
        y = np.linspace(point_list_y[i], point_list_y[i + 1], num=point_nr)
        for j in range(point_nr):
            pose = Pose()
            pose.position.x = x[j]
            pose.position.y = y[j]
            pose_list.append(pose)

    path.poses = pose_list
    pub_path.publish(path)

    # while not rospy.is_shutdown():
    #     # for i in range(point_list_x.size - 2):

    #     input("Press Enter to send PoseArray")
    #     pub_path.publish(path)
    #     rate.sleep()


def tf_callback(transform: TFMessage):
    for tf in transform.transforms:
        if 1:
            translation = tf.transform.translation


if __name__ == "__main__":
    try:
        generate_path()
    except rospy.ROSInterruptException:
        pass
