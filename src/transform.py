#!/usr/bin/env python3.8

import rospy
import geometry_msgs.msg
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_euler


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
    rotation_list = rotation

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


def transfrom():
    print("dx")
    # Example usage
    point = geometry_msgs.msg.Pose()
    point.position.x = 1.0
    point.position.y = 2.0
    point.position.z = 3.0

    translation = geometry_msgs.msg.Vector3()
    translation.x = 5.0
    translation.y = -2.0
    translation.z = 1.0

    rotation = geometry_msgs.msg.Quaternion()
    # Assuming you have the Euler angles for rotation
    rotation_euler = [1.0, 0.0, 0.0]  # Replace with your desired rotation
    rotation = quaternion_from_euler(*rotation_euler)

    transformed_point = transform_point(point, translation, rotation)

    print("Original point:", point)
    print("Transformed point:", transformed_point)


if __name__ == "__main__":
    try:
        transfrom()
    except rospy.ROSInterruptException:
        pass
