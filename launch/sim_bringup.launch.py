import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('llm_robot_task_planner')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo resolves package://jetrover_description/
    # The jetrover meshes are installed under <pkg_share>/urdf/jetrover_description/
    gz_resource_path = os.path.join(pkg_dir, 'urdf')
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    combined_path = f'{gz_resource_path}:{existing_gz_path}' if existing_gz_path else gz_resource_path

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_path,
    )

    # Process xacro to URDF
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # World file
    world_file = os.path.join(pkg_dir, 'worlds', 'two_room_world.sdf')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': True}],
    )

    # Launch Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'jetrover',
            '-topic', 'robot_description',
            '-x', '2.0',
            '-y', '2.0',
            '-z', '0.05',
        ],
        output='screen',
    )

    # Bridge Gazebo topics to ROS 2
    # Bridge Gazebo topics to ROS 2
    # DiffDrive subscribes to /cmd_vel and publishes /odom in Gz (not namespaced)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/two_room_world/model/jetrover/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/two_room_world/model/jetrover/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
