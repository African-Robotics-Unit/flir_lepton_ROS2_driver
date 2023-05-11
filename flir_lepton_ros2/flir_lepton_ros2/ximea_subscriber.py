#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from socket import MsgFlag
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge, CvBridgeError

class XimeaSubscriber(Node):


    def __init__(self):
        super().__init__('ximea_subscriber')
        self.subscriber = self.create_subscription(Image, 'ximea/image_raw', self.callback, 10)
        self.image_pub_ = self.create_publisher(Image, 'ximea/rgb_image', 10)
        self.bridge = CvBridge()



    def callback(self, Image):
        self.get_logger().info("Recieving raw data")

        #cv bridge
        msg_fmt = "mono8"
        cv_img = self.bridge.imgmsg_to_cv2(Image, msg_fmt)
        #gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        color_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB)
       
        #add data to image message 
        msg = self.bridge.cv2_to_imgmsg(color_img, "rgb8")
        msg.header.stamp = Image.header.stamp


        # Publish Gray Scale Img for visualization
        try:
            self.image_pub_.publish(msg)
        except CvBridgeError as e:
            print(e)

        self.get_logger().info("Published color image")



def main(args=None):
    rclpy.init(args=args)

    nh = XimeaSubscriber()
    rclpy.spin(nh)

    nh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
