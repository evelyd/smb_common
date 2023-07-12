#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_matrix, concatenate_matrices
from geometry_msgs.msg import TransformStamped
import numpy as np

camera_to_world_matrix = None
camera_to_world_pub = None

def tf_callback(msg):
    global camera_to_world_matrix

    # Extract the translation and rotation from the /tf message
    translation = msg.transform.translation
    rotation = msg.transform.rotation

    # Create a transformation matrix for the position and orientation
    translation_matrix = tf.transformations.translation_matrix(
        [translation.x, translation.y, translation.z]
    )
    rotation_matrix = euler_from_quaternion([rotation.x, rotation.y, rotation.z , rotation.w])

    # Combine translation and rotation matrices
    tf_matrix = concatenate_matrices(translation_matrix, rotation_matrix)

    # Store the camera-to-world transformation matrix
    camera_to_world_matrix = tf_matrix

def detected_poses_callback(msg):
    global camera_to_world_matrix

    for pose in msg.poses:
        # Extract the translation and rotation from the detected poses
        translation = pose.position
        rotation = pose.orientation

        # Create a transformation matrix for the position and orientation
        translation_matrix = tf.transformations.translation_matrix(
            [translation.x, translation.y, translation.z]
        )
        rotation_matrix = euler_from_quaternion([rotation.x, rotation.y, rotation.z , rotation.w])

   

        # Combine translation and rotation matrices
        pose_matrix = concatenate_matrices(translation_matrix, rotation_matrix)

        if camera_to_world_matrix is not None:
            # Multiply pose matrix with camera-to-world transformation
            final_matrix = np.matmul(camera_to_world_matrix, pose_matrix)

            # Publish the camera-to-world transformation
            publish_camera_to_world(final_matrix)

def publish_camera_to_world(matrix):
    global camera_to_world_pub

    # Create a TransformStamped message
    transform_msg = TransformStamped()
    transform_msg.header.stamp = rospy.Time.now()
    transform_msg.header.frame_id = 'camera_frame'
    transform_msg.child_frame_id = 'world_frame'

    # Extract translation from the matrix
    translation = tf.transformations.translation_from_matrix(matrix)
    transform_msg.transform.translation.x = translation[0]
    transform_msg.transform.translation.y = translation[1]
    transform_msg.transform.translation.z = translation[2]

    # Extract rotation from the matrix
    rotation = tf.transformations.quaternion_from_matrix(matrix)
    transform_msg.transform.rotation.x = rotation[0]
    transform_msg.transform.rotation.y = rotation[1]
    transform_msg.transform.rotation.z = rotation[2]
    transform_msg.transform.rotation.w = rotation[3]

    # Publish the camera-to-world transformation
    camera_to_world_pub.publish(transform_msg)

def main():
    rospy.init_node('pose_conversion_node')

    global camera_to_world_pub
    camera_to_world_pub = rospy.Publisher('/camera_to_world', TransformStamped, queue_size=10)

    # Subscribe to the /tf topic
    tf_sub = rospy.Subscriber('/tf', tfMessage, tf_callback)

    # Subscribe to the /object_detector/detected_poses topic
    detected_poses_sub = rospy.Subscriber('/object_detector/detected_poses', PoseStamped, detected_poses_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
