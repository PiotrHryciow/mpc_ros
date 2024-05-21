#!/usr/bin/env python3.8

import geometry_msgs.msg
import rospy
from geometry_msgs.msg import PoseArray, Vector3, Pose, Quaternion, PoseStamped
import geometry_msgs

# from tf.msg import tfMessage
import numpy as np

import tf2_ros
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_matrix, quaternion_from_euler

# import tf2_geometry_msgs

import matplotlib.pyplot as plt

import tf2_ros.buffer
import tf2_ros.transform_listener


def transform_point(point, translation, rotation):
    """
    Transforms a point (Pose) to another frame using translation and rotation.

    Args:
        point: A geometry_msgs.msg.Pose object representing the point to transform.
        translation: A geometry_msgs.msg.Vector3 object representing the translation vector.
        rotation: A geometry_msgs.msg.Quaternion object representing the rotation quaternion.

    Returns:
        A new geometry_msgs.msg.Pose object representing the transformed point.
    """

    # Extract the position from the point
    position = np.array([point.position.x, point.position.y, point.position.z, 1.0])

    # Convert Quaternion object to a list
    rotation_list = [rotation.x, rotation.y, rotation.z, rotation.w]

    # Create translation matrix
    translation_matrix = np.eye(4)
    translation_matrix[0, 3] = translation.x
    translation_matrix[1, 3] = translation.y
    translation_matrix[2, 3] = translation.z

    # Create rotation matrix from quaternion
    rotation_matrix = quaternion_matrix(rotation_list)

    # Combine rotation and translation into a single transformation matrix
    transformation_matrix = np.dot(translation_matrix, rotation_matrix)

    # Apply the transformation to the position
    transformed_position = np.dot(np.linalg.inv(transformation_matrix), position)

    # Create the transformed pose
    transformed_pose = geometry_msgs.msg.Pose()
    transformed_pose.position.x = transformed_position[0]
    transformed_pose.position.y = transformed_position[1]
    transformed_pose.position.z = transformed_position[2]

    # You can optionally set the orientation of the transformed point here
    # transformed_pose.orientation = rotation  # Assuming the original orientation is preserved

    return transformed_pose


def generate_path():
    rospy.init_node("path_gen", anonymous=True)

    # header = geometry_msgs.msg.
    # header.stamp = rospy.Time()  # Get the current time
    # header.frame_id = source_frame

    original_pose_stamped = PoseStamped()
    # original_pose_stamped.header = header

    pub_path = rospy.Publisher("/set_path", PoseArray, queue_size=10)
    sub_tf = rospy.Subscriber("/tf", TFMessage, tf_callback)
    # point_list_x = np.array(
    #     [-0.16, 0.5, 0.96, 1.37, -0.044, -0.71, -2.89, -3.47, -3.2, -2.7, 0.32]
    # )
    # point_list_y = np.array(
    #     [4.7, 1.3, -0.64, -3.44, -4.92, -4.83, -4.72, -1.93, -0.66, 0.033, 0.417]
    # )
    point_list_x = np.array([-0.16, 0.5])
    point_list_y = np.array([4.7, 1.3])
    # point_list_x = np.array([-1.07, -0.26, 0.655, 0.878, 1.052])
    # point_list_y = np.array([0.348, 0.388, -0.014, -1.03, -2.41])
    # point_list_x = np.array([0.5, 0.58])
    # point_list_y = np.array([0.89, 0.111])
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
            print(pose)
            # print(trans)
            pose = transform_point(pose, trans, rot)
            print(pose)
            # pose.position = translation * pose.position
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
        if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
            print("dostalem")
            global trans, rot
            # print(tf.transform.translation)
            trans = tf.transform.translation
            # print(trans)
            rot = tf.transform.rotation
            # print(translation)


if __name__ == "__main__":
    try:
        trans = Vector3()
        rot = Quaternion()
        generate_path()
    except rospy.ROSInterruptException:
        pass
