#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime
from window_recorder.recorder import WindowRecorder
import time

rospy.init_node('record_rviz', anonymous=True)
file_name = rospy.get_param('~file_name', 'output')
file_name = file_name + "rviz"
with WindowRecorder( window_names=["occ_map_v2.rviz - RViz", "occ_map_v2.rviz* - RViz"], frame_rate=30.0,
    save_dir="/home/vittorio/ros_ws/video_dumps", name_suffix=file_name,record=True):
    start = time.time()
    i = 1
    while not rospy.is_shutdown():
        time.sleep(0.1)