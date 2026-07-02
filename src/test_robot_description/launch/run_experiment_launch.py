import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('test_robot_description')
    world_path = os.path.join(pkg_dir, 'worlds', 'training_world_2.sdf')
    robot_sdf_path = os.path.join(pkg_dir, 'models', 'robot.sdf')
    slam_params_path = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    # =========================================================================
    # WORLD SDF PATCH
    # training_world_2.sdf is missing gz-sim-sensors-system (competition file,
    # cannot be edited). We read it, inject the plugin in memory, and write it
    # to a temp file so Gazebo picks it up — no edits to the original file.
    # =========================================================================
    with open(world_path, 'r') as f:
        world_sdf = f.read()

    sensors_plugin = (
        '\n    <plugin filename="gz-sim-sensors-system"'
        ' name="gz::sim::systems::Sensors">'
        '\n      <render_engine>ogre2</render_engine>'
        '\n    </plugin>'
    )
    # Insert right before </world> closing tag
    world_sdf = world_sdf.replace('  </world>', sensors_plugin + '\n  </world>')

    # Write patched world to a temp file that persists for the process lifetime
    tmp_world = tempfile.NamedTemporaryFile(
        mode='w', suffix='_training_world_2_patched.sdf', delete=False
    )
    tmp_world.write(world_sdf)
    tmp_world.flush()
    patched_world_path = tmp_world.name

    # =========================================================================
    # ROBOT SDF PATCHES (Indentation-Proof Version)
    # =========================================================================
    with open(robot_sdf_path, 'r') as f:
        sdf = f.read()

    # PATCH 1 – Disable self-collision
    sdf = sdf.replace(
        '<model name="diff_drive_robot">',
        '<model name="diff_drive_robot">\n    <self_collide>false</self_collide>'
    )

    # PATCH 2 – Fix wheel joint axes (Targeting the unique xyz tag inside the axes)
    sdf = sdf.replace(
        '<xyz>0 0 1</xyz>',
        '<xyz expressed_in="__model__">0 1 0</xyz>\n        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>'
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

    # PATCH 6 – Inject explicit topic names into DiffDrive plugin (Targeting just the header line)
    sdf = sdf.replace(
        '<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">',
        """<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <topic>/cmd_vel</topic>
        <odom_topic>/odom</odom_topic>
        <tf_topic>/tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>"""
    )

    # PATCH 7 – Inject <topic> into lidar sensor
    sdf = sdf.replace(
        '<sensor name="gpu_lidar" type="gpu_lidar">',
        '<sensor name="gpu_lidar" type="gpu_lidar">\n        <topic>/scan</topic>'
    )

    # =========================================================================
    # LAUNCH NODES
    # =========================================================================

    # 1. Gazebo Sim — loads the patched temp world (has sensors system injected)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r {patched_world_path}'}.items()
    )

    # 2. Spawn robot — z=0.1 so wheels clear the ground on spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', sdf,
            '-name', 'robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
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
        remappings=[
            ('/cmd_vel', '/robot/cmd_vel'),
            ('/odom',    '/robot/odom'),
            ('/scan',    '/robot/scan'),
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

    # 5. slam_toolbox — delayed start
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': 'true'
        }.items()
    )

    slam_delayed = TimerAction(
        period=5.0,
        actions=[slam_toolbox_launch]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        parameter_bridge,
        lidar_static_tf,
        slam_delayed,
    ])
