#!/usr/bin/env python3

# ^tells Linux to use python 3 to run this file
# when i run it in the terminal, i write: python3 aruco_detection/aruco.py

import rclpy                    # ROS2's python library
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2

### OpenCV (Open source computer vision library) 
# lets us process camera images - and sees "is there an ArUco marker
# in this image?" - has build in functionality specifically for ArUco markers
# and has pose-estimation
# I have version 4.5.4

### CvBridge translates from ROS images to OpenCV images

from sensor_msgs.msg import Image


class ArucoDetector(Node): 

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge() # our ros -> openCv translator

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)

        self.parameters = cv2.aruco.DetectorParameters_create()

        self.image_subscriber = self.create_subscription( # subscribe my node to  topic
            Image,                      # the messages here are sensor_msgs/Image messages
            '/camera/image_raw',        # the topic
            self.image_callback,        # whenever a new image arrives, run image_callback fct
            10                          # can keep up to 10 messages queued
        )
        # my ArucoDetector ROS2 node is subscribed to the camera's ROS topic /camera/image_raw

        self.get_logger().info('ArUco detector started!')
        self.get_logger().info('Listening to /camera/image_raw')

    def image_callback(self, msg):  # runs every time a new camera image arrives
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # translates from ROS to OpenCV here

        corners, ids, rejected = cv2.aruco.detectMarkers(
            image,
            self.dictionary,
            parameters = self.parameters
        )
        # giving OpenCV image + dictionary + detector settings and asking if it finds any markers in the image
        # returns the IDs, corners (four pixel coord. of the markers corners), rejected - not using right now

        if ids is not None:
            self.get_logger().info(
                f'Detected ArUco marker: {ids.flatten().tolist()}'
            )

        #self.get_logger().info(
            #f'Received an image: {image.shape}')
        # printing on every single frame, even when there isn't a marker


def main(args=None):
    rclpy.init(args=args) # starts the ROS 2 python system

    node = ArucoDetector() # creates detector

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
