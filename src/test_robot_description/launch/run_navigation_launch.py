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
    map_yaml_path = '/home/s43/test_sensor_ws/my_first_map.yaml'

    # =========================================================================
    # WORLD SDF PATCH
    # =========================================================================
    with open(world_path, 'r') as f:
        world_sdf = f.read()

    # PATCH A — flip all dynamic obstacles to static (fixes RTF=4%)
    world_sdf = world_sdf.replace('<static>false</static>', '<static>true</static>')

    # PATCH B — relax physics: 1ms→4ms, 1000Hz→250Hz
    world_sdf = world_sdf.replace(
        '<max_step_size>0.001</max_step_size>',
        '<max_step_size>0.004</max_step_size>'
    )
    world_sdf = world_sdf.replace(
        '<real_time_update_rate>1000</real_time_update_rate>',
        '<real_time_update_rate>250</real_time_update_rate>'
    )

    # PATCH C — inject missing sensors system plugin
    sensors_plugin = (
        '\n    <plugin filename="gz-sim-sensors-system"'
        ' name="gz::sim::systems::Sensors">'
        '\n      <render_engine>ogre2</render_engine>'
        '\n    </plugin>'
    )
    world_sdf = world_sdf.replace('  </world>', sensors_plugin + '\n  </world>')

    tmp_world = tempfile.NamedTemporaryFile(
        mode='w', suffix='_training_world_2_patched.sdf', delete=False
    )
    tmp_world.write(world_sdf)
    tmp_world.flush()
    patched_world_path = tmp_world.name

    # =========================================================================
    # ROBOT SDF PATCHES
    # =========================================================================
    with open(robot_sdf_path, 'r') as f:
        sdf = f.read()

    sdf = sdf.replace(
        '<model name="diff_drive_robot">',
        '<model name="diff_drive_robot">\n    <self_collide>false</self_collide>'
    )
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
    sdf = sdf.replace(
        '<joint name="caster_joint" type="ball">',
        '<joint name="caster_joint" type="fixed">'
    )
    sdf = sdf.replace(
        '<pose>-0.2 0 -0.08 0 0 0</pose>',
        '<pose>-0.2 0 -0.05 0 0 0</pose>'
    )
    sdf = sdf.replace(
        '<link name="left_wheel">\n      <pose>0.15 0.225 -0.025 -1.5707 0 0</pose>',
        '<link name="left_wheel">\n      <pose>0.15 0.225 0.0 -1.5707 0 0</pose>'
    )
    sdf = sdf.replace(
        '<link name="right_wheel">\n      <pose>0.15 -0.225 -0.025 -1.5707 0 0</pose>',
        '<link name="right_wheel">\n      <pose>0.15 -0.225 0.0 -1.5707 0 0</pose>'
    )
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
    sdf = sdf.replace(
        '<sensor name="gpu_lidar" type="gpu_lidar">',
        '<sensor name="gpu_lidar" type="gpu_lidar">\n        <topic>/scan</topic>'
    )

    # =========================================================================
    # LAUNCH NODES
    # =========================================================================

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {patched_world_path}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', sdf, '-name', 'robot', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

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

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': 'true'
        }.items()
    )

    nav2_delayed = TimerAction(period=5.0, actions=[nav2_launch])

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        parameter_bridge,
        lidar_static_tf,
        nav2_delayed,
    ])
