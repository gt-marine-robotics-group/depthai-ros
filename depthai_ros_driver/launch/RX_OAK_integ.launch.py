from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    left_cam_info_sub = Node(
        package='depthai_ros_driver',
        executable='RX_OAK_sub_node.py',
        name='OAK_cam_info_sub',
        namespace='left_cam'
    )
    right_cam_info_sub = Node(
        package='depthai_ros_driver',
        executable='RX_OAK_sub_node.py',
        name='OAK_cam_info_sub',
        namespace='right_cam'
    )
    launch_dir = os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch')
    # Run the node
    return LaunchDescription([
        left_cam_info_sub,
        # right_cam_info_sub,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'RX_camera.launch.py')),
            # launch_arguments={'tf_prefix': name,
            #                   'camera_model': camera_model,
            #                   'base_frame': name,
            #                   'parent_frame': parent_frame,
            #                   'cam_pos_x': cam_pos_x,
            #                   'cam_pos_y': cam_pos_y,
            #                   'cam_pos_z': cam_pos_z,
            #                   'cam_roll': cam_roll,
            #                   'cam_pitch': cam_pitch,
            #                   'cam_yaw': cam_yaw,
            #                   'use_composition': use_composition,
            #                   'use_base_descr': publish_tf_from_calibration}.items()
            ),

    ])