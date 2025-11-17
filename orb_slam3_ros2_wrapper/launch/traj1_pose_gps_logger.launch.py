from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='orb_slam3_ros2_wrapper',
            executable='traj1_pose_gps_logger.py',
            name='traj1_pose_gps_logger',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'pose_topic': '/robot/robot_pose_slam',
                'gps_topic': '/world/z_simple_palm_plantation/model/x500_custom_0/link/gps_link/sensor/gps/navsat',
                'odometry_topic': '/model/x500_custom_0/odometry_with_covariance',
            }]
        )
    ])


