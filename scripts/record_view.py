#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime


def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Write image to video file
    video_writer.write(cv_image)

if __name__ == '__main__':
    rospy.init_node('image_to_video_node')

    # Get file name from rosparam
    image_topic = 'image'
    file_name = rospy.get_param('~file_name', 'output')
    base_path = "/home/vittorio/ros_ws/video_dumps/" 
    current_date = datetime.datetime.now().strftime("%y_%m_%d_%H%M%S")
    full_path = base_path + current_date + "_" + file_name + ".mp4"
    codec = rospy.get_param('~codec', 'mp4v')
    codec = cv2.VideoWriter_fourcc(*codec)
    # Create video writer
    print(f"Recording to {full_path}")
    print(f"Recording started at {datetime.datetime.now()}")
    video_writer = cv2.VideoWriter(full_path, codec, 30, (1920, 1200))
    # Subscribe to image topic
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()

    # Release video writer
    video_writer.release()

    # Print recording ended message with timestamp
    print(f"Recording ended at {datetime.datetime.now()}")
