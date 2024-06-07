from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config=get_package_share_directory('aglidar_sdk')+'/rviz/rviz2.rviz'

    return LaunchDescription([
        Node(namespace='aglidar_sdk', package='aglidar_sdk', executable='aglidar_sdk_node', output='screen'),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
