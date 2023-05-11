#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from socket import MsgFlag
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge, CvBridgeError
from flir_boson_interfaces.msg import ThermalRaw


class FlirBosonSubscriber(Node):


    def __init__(self):
        super().__init__('flir_boson_subscriber')
        self.subscriber = self.create_subscription(ThermalRaw, 'boson/thermal_raw', self.callback, 10)
        self.image_pub_ = self.create_publisher(Image, 'boson/image_raw', 10)

        self.bridge = CvBridge()


    def callback(self, ThermalRaw):
        self.get_logger().info("Recieving raw data")
        temp = np.array(ThermalRaw.data).reshape(ThermalRaw.height, ThermalRaw.width)
        # Raw Sensor Data to GrayScale Image
        img = raw_to_8bit(temp)

        #add data to image message 
        msg = self.bridge.cv2_to_imgmsg(img, "rgb8")
        msg.header.stamp = ThermalRaw.header.stamp


        # Publish Gray Scale Img for visualization
        try:
            self.image_pub_.publish(msg)
        except CvBridgeError as e:
            print(e)

        self.get_logger().info("Published gray image")
    
def raw_to_8bit(data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)



def main(args=None):
    rclpy.init(args=args)

    nh = FlirBosonSubscriber()
    rclpy.spin(nh)

    nh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
