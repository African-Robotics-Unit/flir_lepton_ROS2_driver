#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from flir_lepton_interfaces.msg import ThermalRaw
from .submodules.uvctypes import *
import numpy as np

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class FlirLepton(object):


    def __init__(self, on_thermal, serial_num, logger=None):

        self.on_thermal = on_thermal
        self.serial_num = serial_num
        self.is_active = False

        self.ctx = POINTER(uvc_context)()
        self.devs = POINTER(c_void_p)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()

        self.ptr_py_frame_callback = CFUNCTYPE(
            None, POINTER(uvc_frame), c_void_p)(self.py_frame_callback)

        res = libuvc.uvc_init(byref(self.ctx), 0)
        if res < 0:
            print('uvc_init error')
            exit(1)
        else:
            print('uvc_init success')
        
        
        self.devs = self.get_devices(self.ctx)
        
        if len(self.devs) == 0:
            print("Did not find any devices")
            exit(1)
        else:
            print("Found {} devices".format(len(self.devs)))

        for (desc, dev) in self.devs:
            if str(desc.serialNumber) == str(self.serial_num.value):
                self.dev = dev
                print(str(self.serial_num.value))

        res = libuvc.uvc_open(self.dev, byref(self.devh))
        if res < 0:
            print('uvc_open error')
            exit(1)
        else:
            print('Flir lepton device opened!')

        print_device_info(self.devh)
        print_device_formats(self.devh)

        frame_formats = uvc_get_frame_formats_by_guid(
            self.devh, VS_FMT_GUID_Y16)

        if len(frame_formats) == 0:
            print('device does not support Y16')
            exit(1)

        libuvc.uvc_get_stream_ctrl_format_size(
            self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_Y16,
            frame_formats[0].wWidth, frame_formats[0].wHeight,
            int(1e7 / frame_formats[0].dwDefaultFrameInterval)
        ) 


    def get_devices(self, ctx):

        devices = []
        devs_temp = POINTER(c_void_p)()
        res = libuvc.uvc_get_device_list(self.ctx, byref(devs_temp))
        
        if res != 0:
            print("uvc_find_device error")
            exit(1)
        else:
            print('uvc_find_devices successful')

        count = 0
        while devs_temp[count] != None:
            dev_temp = cast(devs_temp[count], POINTER(uvc_device))
            count += 1

            desc = POINTER(uvc_device_descriptor)()
            res = libuvc.uvc_get_device_descriptor(dev_temp, byref(desc))

            if res != 0:
                self("Could not get device descriptor")
                continue

            if desc.contents.idProduct == PT_USB_PID and desc.contents.idVendor == PT_USB_VID:
                devices.append((desc.contents, dev_temp))

        return devices


    
    def py_frame_callback(self, frame, userptr, copy=False):

        if copy:
            # copy
            data = np.fromiter(
            frame.contents.data, dtype=np.dtype(np.uint8), count=frame.contents.data_bytes
            ).reshape(
            frame.contents.height, frame.contents.width, 2
            ) 
        else:
            # no copy
            array_pointer = cast(frame.contents.data, POINTER(
                c_uint16 * (frame.contents.width * frame.contents.height)))
            data = np.frombuffer(
                array_pointer.contents, dtype=np.dtype(np.uint16)
            ).reshape(
                frame.contents.height, frame.contents.width
            )

        if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
            return

        self.on_thermal(data)

    def start(self):
        if not self.is_active:
            res = libuvc.uvc_start_streaming(
                self.devh, byref(self.ctrl), self.ptr_py_frame_callback, None, 0)
            if res < 0:
                print('uvc_start_streaming failed: {0}'.format(res))
                exit(1)
            else:
                self.is_active = True
                print('uvc_start_streaming success')

    def stop(self):
        if self.is_active:
            libuvc.uvc_stop_streaming(self.devh)
            self.is_active = False
            print('uvc_stop_streaming')

        libuvc.uvc_unref_device(self.dev)
        libuvc.uvc_exit(self.ctx)


class FlirLeptonPublisher(Node):


    def __init__(self):
        super().__init__('flir_lepton_publisher')
        self.declare_parameter('serial_num')
        self.serial_num = self.get_parameter('serial_num')
        self.get_logger().info('serial_num:  {0}'.format(str(self.serial_num.value)))

        self.frame_count = 0
        self.publisher = self.create_publisher(ThermalRaw, 'lepton/thermal_raw', 10)
        self.image_pub_ = self.create_publisher(Image, 'lepton/image_raw', 10)

        self.flirlepton = FlirLepton(self.on_thermal,  self.serial_num, logger=self.get_logger())
        self.flirlepton.start()

    def on_thermal(self, data):
        self.get_logger().info("trying to publish raw data.")
        
        msg = ThermalRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_count)
        self.frame_count+=1 
        msg.height, msg.width = data.shape
        msg.data = data.ravel().tolist()
        self.publisher.publish(msg)
        self.get_logger().info("published raw data success.")

        #publish false colour image as ROS2 Image msg to view
        data = np.array(msg.data).reshape(msg.height, msg.width)

        try:
            self.bridge = CvBridge()
        except CvBridgeError as e:
            print(e)

        # Raw Sensor Data to GrayScale Image
        cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
        np.right_shift(data, 8, data)
        img_msg = cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)
        img_msg = ~img_msg

        #add data to image message 
        img_msg = self.bridge.cv2_to_imgmsg(img_msg, "rgb8")
        img_msg.header.stamp = msg.header.stamp

        # Publish Gray Scale Img for visualization
        try:
            self.image_pub_.publish(img_msg)
        except CvBridgeError as e:
            print(e)


    def on_shutdown(self):
        self.get_logger().info('shutting down process')
        try:
            self.flirlepton.stop()
        except:
            pass
        finally:
            self.flirlepton = None

        



def main(args=None):
    rclpy.init(args=args)
    nh = FlirLeptonPublisher()
    
    try:
        rclpy.spin(nh)
    except KeyboardInterrupt:
        nh.get_logger().info('KeyboardInterrupt')
    except Exception as e:
        nh.get_logger().error(e)
    finally:
        nh.on_shutdown()
        nh.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
