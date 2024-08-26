#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class CamInfoSubscriber(Node):

    def __init__(self):
        super().__init__('cam_info_subscriber')

        # Define a QoS profile with RELIABLE reliability policy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, # keep the lastest 10
            depth=10  # max number of msg stored in a subscription
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/oak/points',
            self.listener_callback,
            qos_profile)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received point cloud data')


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