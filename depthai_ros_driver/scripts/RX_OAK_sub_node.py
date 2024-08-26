#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from theora_image_transport.msg import Packet
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

        self.create_subscription(
            Imu,
            '/oak/imu/data',
            self.listener_callback_imu,
            10)
        
        self.create_subscription(
            PointCloud2,
            '/oak/points',
            self.listener_callback_points,
            qos_profile)
        
        self.create_subscription(
            Detection3DArray,
            '/oak/nn/spatial_detections',
            self.listener_callback_nn,
            10)
        
        self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.listener_callback_rgb_cam_info,
            10)
        
        self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.listener_callback_rgb_raw,
            10)
        
        self.create_subscription(
            CompressedImage,
            '/oak/rgb/image_raw/comporessed',
            self.listener_callback_rgb_c,
            10)
        
        self.create_subscription(
            CompressedImage,
            '/oak/rgb/image_raw/comporessedDepth',
            self.listener_callback_rgb_cd,
            10)
        
        self.create_subscription(
            Packet,
            '/oak/rgb/image_raw/theora',
            self.listener_callback_rgb_theora,
            10)
        
        self.create_subscription(
            CameraInfo,
            '/oak/stereo/camera_info',
            self.listener_callback_stereo_cam_info,
            10)
        
        self.create_subscription(
            Image,
            '/oak/stereo/image_raw',
            self.listener_callback_stereo_raw,
            10)
        
        self.create_subscription(
            CompressedImage,
            '/oak/stereo/image_raw/comporessed',
            self.listener_callback_stereo_c,
            10)
        
        self.create_subscription(
            CompressedImage,
            '/oak/stereo/image_raw/comporessedDepth',
            self.listener_callback_stereo_cd,
            10)
        
        self.create_subscription(
            Packet,
            '/oak/stereo/image_raw/theora',
            self.listener_callback_stereo_theora,
            10)

    def listener_callback_imu(self, msg):
        self.get_logger().info('Received IMU data')

    def listener_callback_points(self, msg):
        self.get_logger().info('Received point cloud data')

    def listener_callback_nn(self, msg):
        self.get_logger().info('Received NN spatial detections')

    def listener_callback_rgb_cam_info(self, msg):
        self.get_logger().info('Received RGB camera info')

    def listener_callback_rgb_raw(self, msg):
        self.get_logger().info('Received RGB image raw')

    def listener_callback_rgb_c(self, msg):
        self.get_logger().info('Received RGB image compressed')

    def listener_callback_rgb_cd(self, msg):
        self.get_logger().info('Received RGB image compressed depth')

    def listener_callback_rgb_theora(self, msg):
        self.get_logger().info('Received RGB image theora')

    def listener_callback_stereo_cam_info(self, msg):
        self.get_logger().info('Received stereo camera info')

    def listener_callback_stereo_raw(self, msg):
        self.get_logger().info('Received stereo image raw')

    def listener_callback_stereo_c(self, msg):
        self.get_logger().info('Received stereo image compressed')

    def listener_callback_stereo_cd(self, msg):
        self.get_logger().info('Received stereo image compressed depth')

    def listener_callback_stereo_theora(self, msg):
        self.get_logger().info('Received stereo image theora')


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