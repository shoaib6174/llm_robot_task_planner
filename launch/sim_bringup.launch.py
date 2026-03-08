import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('llm_robot_task_planner')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo resolves package://jetrover_description/
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

    # Paths
    world_file = os.path.join(pkg_dir, 'worlds', 'two_room_world.sdf')
    map_file = os.path.join(pkg_dir, 'maps', 'two_room_map.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Rewrite params with sim time
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'use_sim_time': 'True'},
        convert_types=True,
    )

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
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/two_room_world/model/jetrover/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Arm joint position commands (ROS → Gazebo)
            '/arm/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/joint4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/joint5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/gripper/r_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/gripper/l_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/world/two_room_world/model/jetrover/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    # Convert /odom to odom->base_footprint TF
    # (We don't bridge Gazebo /tf because it contains ALL model poses,
    # which conflicts with robot_state_publisher's joint TFs.)
    odom_to_tf = Node(
        package='llm_robot_task_planner',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ==================== Nav2 Nodes ====================
    # Launched directly instead of via nav2_bringup to avoid
    # unused nodes (docking_server, route_server) that fail to configure.

    nav2_lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_file}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': nav2_lifecycle_nodes,
            'bond_timeout': 0.0,
        }],
    )

    # ==================== Perception ====================
    perception_node = Node(
        package='llm_robot_task_planner',
        executable='perception_node',
        name='perception_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ==================== Arm Controller ====================
    arm_controller = Node(
        package='llm_robot_task_planner',
        executable='arm_controller',
        name='arm_controller',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ==================== World Model ====================
    world_model = Node(
        package='llm_robot_task_planner',
        executable='world_model',
        name='world_model',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ==================== LLM Agent ====================
    # Not launched by default — run separately so OPENAI_API_KEY can be set.
    # To launch: ros2 run llm_robot_task_planner llm_agent

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        odom_to_tf,
        # Nav2 nodes
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        # Perception
        perception_node,
        # Arm
        arm_controller,
        # World model
        world_model,
    ])
