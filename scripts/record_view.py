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
    file_name = rospy.get_param('~file_name', 'output.mp4')
    base_path = "/home/vittorio/ros_ws/video_dumps/" 
    codec = rospy.get_param('~codec', 'mp4v')
    codec = cv2.VideoWriter_fourcc(*codec)
    # Create video writer
    video_writer = cv2.VideoWriter(base_path + file_name, codec, 30, (1920, 1200))
    # Print recording started message with timestamp
    print(f"Recording started at {datetime.datetime.now()}")

    # Subscribe to image topic
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()

    # Release video writer
    video_writer.release()

    # Print recording ended message with timestamp
    print(f"Recording ended at {datetime.datetime.now()}")
