#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from socket import MsgFlag
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from flir_boson_interfaces.msg import ThermalRaw


class FlirBosonSubscriber(Node):


    def __init__(self):
        super().__init__('flir_boson_subscriber')
        self.img = []
        self.heatmap = []
        self.raw_arr = []
        self.raw_arr_temp = []
        self.image_window = 'FLIR Boson'
        cv2.namedWindow(self.image_window)
        cv2.setMouseCallback(self.image_window, self.mouse_callback)
        
        self.raw_subscriber = self.create_subscription(ThermalRaw, 'boson/thermal_raw', self.raw_callback, 10)
        self.img_subscriber = self.create_subscription(Image, 'boson/image_raw', self.img_callback, 10)

        self.bridge = CvBridge()


    def raw_callback(self, ThermalRaw):
        self.raw_arr = np.array(ThermalRaw.data).reshape(ThermalRaw.height, ThermalRaw.width)

    
    def img_callback(self, Image):
        msg_fmt = "mono8"
        self.img = self.bridge.imgmsg_to_cv2(Image, msg_fmt)
        cv2.cvtColor(self.img, cv2.COLOR_GRAY2RGB)
        cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        self.img = ~self.img
        self.heatmap = cv2.applyColorMap(self.img, cv2.COLORMAP_HOT)
        cv2.imshow(self.image_window, self.heatmap)

        cv2.waitKey(1)
     
    
    def raw_to_8bit(self, data):
        cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
        np.right_shift(data, 8, data)
        return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

    def mouse_callback(self, event, x, y, flags, params):

        # checking for right mouse clicks     
        if event==cv2.EVENT_LBUTTONDOWN:
            # displaying the coordinates
            temp = self.raw_to_c(self.raw_arr[y,x])
            font = cv2.FONT_HERSHEY_SIMPLEX
            print(temp)
            cv2.putText(self.heatmap, "{0:.1f} C".format(temp),
                        (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.imshow(self.image_window, self.heatmap)

    def raw_to_c(self, val):
        return (val - 27315) / 100.0


def main(args=None):
    rclpy.init(args=args)

    nh = FlirBosonSubscriber()
    rclpy.spin(nh)

    nh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()