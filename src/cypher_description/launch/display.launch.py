from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Absolute path to your URDF
    pkg_path = get_package_share_directory('cypher_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'cypher_body.urdf')

    # Read URDF contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Launch Ignition Gazebo (Fortress)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
  # verbose logging
            output='screen'
        ),

        # Publish robot_state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_pub',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # Spawn the robot in Ignition Gazebo
        Node(
            package='ros_ign_gazebo',
            executable='create',
            name='spawn_robot',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'cypher']
        ),
    ])
