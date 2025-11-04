from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('cypher_v0')
    urdf_file = os.path.join(pkg_path, 'cypher_description', 'urdf', 'cypher_body.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Launch Ignition Gazebo Fortress with empty world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', 'empty.sdf'],
            output='screen'
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_pub',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # Spawn robot
        Node(
            package='ros_ign_gazebo',
            executable='create',
            name='spawn_robot',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'cypher']
        ),
    ])
