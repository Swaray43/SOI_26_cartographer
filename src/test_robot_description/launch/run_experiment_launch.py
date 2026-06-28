import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_dir = get_package_share_directory('test_robot_description')
    world_path = os.path.join(pkg_dir, 'worlds', 'training_world_3.sdf')
    robot_sdf_path = os.path.join(pkg_dir, 'models', 'robot.sdf')
    slam_params_path = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    with open(robot_sdf_path, 'r') as f:
        sdf = f.read()

    # =========================================================================
    # RUNTIME SDF PATCHES
    # =========================================================================

    # PATCH 1 – Disable self-collision
    sdf = sdf.replace(
        '<model name="diff_drive_robot">',
        '<model name="diff_drive_robot">\n    <self_collide>false</self_collide>'
    )

    # PATCH 2 – Fix wheel joint axes (primary flip fix)
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
      </axis>"""
    )

    # PATCH 3 – Caster joint: ball → fixed
    sdf = sdf.replace(
        '<joint name="caster_joint" type="ball">',
        '<joint name="caster_joint" type="fixed">'
    )

    # PATCH 4 – Fix caster z-pose
    sdf = sdf.replace(
        '<pose>-0.2 0 -0.08 0 0 0</pose>',
        '<pose>-0.2 0 -0.05 0 0 0</pose>'
    )

    # PATCH 5 – Fix wheel z-poses
    sdf = sdf.replace(
        '<link name="left_wheel">\n      <pose>0.15 0.225 -0.025 -1.5707 0 0</pose>',
        '<link name="left_wheel">\n      <pose>0.15 0.225 0.0 -1.5707 0 0</pose>'
    )
    sdf = sdf.replace(
        '<link name="right_wheel">\n      <pose>0.15 -0.225 -0.025 -1.5707 0 0</pose>',
        '<link name="right_wheel">\n      <pose>0.15 -0.225 0.0 -1.5707 0 0</pose>'
    )

    # PATCH 6 – Inject explicit topic names into DiffDrive plugin
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
    </plugin>"""
    )

    # PATCH 7 – Inject <topic> into lidar sensor
    sdf = sdf.replace(
        '<sensor name="gpu_lidar" type="gpu_lidar">',
        '<sensor name="gpu_lidar" type="gpu_lidar">\n        <topic>/scan</topic>'
    )

    # =========================================================================
    # LAUNCH NODES
    # =========================================================================

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 2. Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', sdf,
            '-name', 'robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # 3. ROS ↔ Gazebo bridge
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # 4. Static TF: base_link → robot/lidar_link/gpu_lidar
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.1',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'robot/lidar_link/gpu_lidar'
        ],
        ros_arguments=['--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # 5. slam_toolbox — delayed start via its official launch file
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': 'true'
        }.items()
    )

    # Wrap the SLAM launch in a GroupAction to apply the topic remapping
    slam_group = GroupAction([
        SetRemap(src='/robot/scan', dst='/scan'),
        slam_toolbox_launch
    ])

    # Pass the slam_group to the TimerAction instead of slam_toolbox_launch
    slam_delayed = TimerAction(
        period=5.0,
        actions=[slam_group]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        parameter_bridge,
        lidar_static_tf,
        slam_delayed,
    ])
