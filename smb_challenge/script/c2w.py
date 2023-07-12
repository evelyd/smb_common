#!/usr/bin/env python
import rospy
import rospkg
import tf
import json
from os.path import join
from geometry_msgs.msg import Point
import tf.transformations as tft
from object_detection_msgs.msg import   PointCloudArray,\
                                        ObjectDetectionInfo, ObjectDetectionInfoArray
import numpy as np

classAndPosition_dict = {}
rospack = rospkg.RosPack()
dict_sace_path = join(rospack.get_path('smb_challenge'),'report')



def detection_info_callback(msg):
    global classAndPosition_dict

    for info in msg.info:
        # lookup tf
        
        if info.position.z<0:
            continue
        try:
            (trans,rot) = listener.lookupTransform('/world_graph_msf', '/rgb_camera_optical_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rotation_matrix = tft.quaternion_matrix(rot)
        translation_matrix = tft.translation_matrix(trans)
        transform_matrix = np.matmul(translation_matrix, rotation_matrix)
        # print(trans, rot)
        position_in_world = np.matmul(transform_matrix,np.array([info.position.x,info.position.y,info.position.z,1]))
        position_in_world = position_in_world[:3]
        print("[position_in_world]:")
        # print(position_in_world)
        if info.class_id not in classAndPosition_dict.keys():
            classAndPosition_dict[info.class_id] = [position_in_world]
        else:
            classAndPosition_dict[info.class_id].append(position_in_world)
        print("[dict]:")
        print(classAndPosition_dict)

def save_file_callback():
    with open(dict_sace_path+'object_detection_dict.json', 'w') as json_file:
        json.dump(classAndPosition_dict, json_file)
    rospy.loginfo("save dict file")

rospy.init_node('c2w_node')

listener = tf.TransformListener()
timer = rospy.Timer(rospy.Duration(10), save_file_callback)


while not rospy.is_shutdown():
    detected_poses_sub = rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, detection_info_callback)
    rospy.spin()