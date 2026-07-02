import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('test_robot_description')
    world_path       = os.path.join(pkg_dir, 'worlds', 'training_world_1.sdf')
    robot_sdf_path   = os.path.join(pkg_dir, 'models', 'robot.sdf')
    slam_params_path = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    with open(robot_sdf_path, 'r') as f:
        sdf = f.read()

    # ── SDF patches ───────────────────────────────────────────────────────
    sdf = sdf.replace('<model name="diff_drive_robot">',
        '<model name="diff_drive_robot">\n    <self_collide>false</self_collide>')

    sdf = sdf.replace(
        """<axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>""",
        """<axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>
      </axis>""")

    sdf = sdf.replace('<joint name="caster_joint" type="ball">',
        '<joint name="caster_joint" type="fixed">')
    sdf = sdf.replace('<pose>-0.2 0 -0.08 0 0 0</pose>',
        '<pose>-0.2 0 -0.05 0 0 0</pose>')
    sdf = sdf.replace(
        '<link name="left_wheel">\n      <pose>0.15 0.225 -0.025 -1.5707 0 0</pose>',
        '<link name="left_wheel">\n      <pose>0.15 0.225 0.0 -1.5707 0 0</pose>')
    sdf = sdf.replace(
        '<link name="right_wheel">\n      <pose>0.15 -0.225 -0.025 -1.5707 0 0</pose>',
        '<link name="right_wheel">\n      <pose>0.15 -0.225 0.0 -1.5707 0 0</pose>')
    sdf = sdf.replace(
        """    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.45</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
    </plugin>""",
        """    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.45</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
        <topic>/cmd_vel</topic>
        <odom_topic>/odom</odom_topic>
        <tf_topic>/tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
    </plugin>""")
    sdf = sdf.replace('<sensor name="gpu_lidar" type="gpu_lidar">',
        '<sensor name="gpu_lidar" type="gpu_lidar">\n        <topic>/scan</topic>')

    # ── Gazebo ────────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-string', sdf, '-name', 'robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )

    parameter_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='sensor_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/cmd_vel', '/robot/cmd_vel'),
            ('/odom',    '/robot/odom'),
            ('/scan',    '/robot/scan'),
        ],
        output='screen'
    )

    # ── TF publisher node (Python, properly uses sim time) ─────────────────
    # ROOT CAUSE OF ALL TF FAILURES:
    # static_transform_publisher ignores use_sim_time in this environment
    # regardless of how it is passed (ros_arguments, parameters, --ros-args).
    # It always stamps /tf_static with wall-clock time (~1.78 billion s).
    # LaserScan carries sim time (~3-70s). TF2 lookup fails → queue full.
    #
    # FIX: Replace static_transform_publisher with a minimal Python node
    # that publishes to /tf (not /tf_static) using the node's own clock.
    # When parameters=[{'use_sim_time': True}] is set on a Python Node(),
    # rclpy correctly subscribes to /clock and all self.get_clock() calls
    # return sim time. The transform is republished every 0.1s so TF2
    # buffer always has a fresh entry matching current sim time.
    lidar_tf_node = Node(
        package='test_robot_description',
        executable='lidar_tf_publisher',
        name='lidar_tf_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── slam_toolbox via official launch ──────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        parameter_bridge,
        lidar_tf_node,
        TimerAction(period=5.0, actions=[slam_launch]),
    ])
