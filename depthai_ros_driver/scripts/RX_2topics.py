#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray


class CamInfoSubscriber(Node):

    def __init__(self):
        super().__init__('cam_info_subscriber')
        
        self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.listener_callback_rgb_raw,
            10)
        
        self.create_subscription(
            Detection3DArray,
            '/oak/nn/spatial_detections',
            self.listener_callback_nn,
            10)

    def listener_callback_rgb_raw(self, msg):
        self.get_logger().info('Received RGB image raw')

    def listener_callback_nn(self, msg):
        self.get_logger().info('Received spatial detections')


def main(args=None):
    rclpy.init(args=args)

    cam_info_subscriber = CamInfoSubscriber()

    rclpy.spin(cam_info_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_info_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()