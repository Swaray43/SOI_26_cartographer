import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('test_robot_description')
    world_path       = os.path.join(pkg_dir, 'worlds', 'training_world_2.sdf')
    robot_sdf_path   = os.path.join(pkg_dir, 'models', 'robot.sdf')
    slam_params_path = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    # =========================================================================
    # WORLD SDF PATCHES
    # =========================================================================
    with open(world_path, 'r') as f:
        world_sdf = f.read()

    # PATCH A — flip all dynamic obstacles to static
    world_sdf = world_sdf.replace('<static>false</static>', '<static>true</static>')

    # PATCH B — relax physics: 1ms→2ms, 1000Hz→500Hz
    world_sdf = world_sdf.replace(
        '<max_step_size>0.001</max_step_size>',
        '<max_step_size>0.002</max_step_size>'
    )
    world_sdf = world_sdf.replace(
        '<real_time_update_rate>1000</real_time_update_rate>',
        '<real_time_update_rate>500</real_time_update_rate>'
    )

    # PATCH C — fix ground plane friction (was empty <ode/> = near-zero friction)
    world_sdf = world_sdf.replace(
        '''        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>''',
        '''        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.5</mu>
                <mu2>1.5</mu2>
                <fdir1>1 0 0</fdir1>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
              </ode>
            </friction>
            <bounce/>
            <contact>
              <ode>
                <min_depth>0.001</min_depth>
                <kp>1e6</kp>
                <kd>100</kd>
              </ode>
            </contact>
          </surface>
        </collision>'''
    )

    # PATCH D — inject missing sensors system plugin
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

    # PATCH 1 – Disable self-collision
    sdf = sdf.replace(
        '<model name="diff_drive_robot">',
        '<model name="diff_drive_robot">\n    <self_collide>false</self_collide>'
    )

    # PATCH 2 – Fix wheel joint axes
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
    # FIX: search string uses 0.45 (matches robot.sdf exactly),
    # replacement sets it to 0.475 (calibrated effective track width).
    # Previously the search used 0.475 which never matched → patch
    # silently failed → no frame_id/tf_topic injected → odom broken.
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
        <wheel_separation>0.475</wheel_separation>
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

    # PATCH 8 – Fix base_link friction
    sdf = sdf.replace(
        """      <collision name="collision">
        <geometry><box><size>0.5 0.4 0.1</size></box></geometry>
        <surface>
          <friction>
            <ode><mu>0.1</mu><mu2>0.1</mu2></ode>
          </friction>
        </surface>
      </collision>""",
        """      <collision name="collision">
        <geometry><box><size>0.5 0.4 0.1</size></box></geometry>
        <surface>
          <friction>
            <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
          </friction>
        </surface>
      </collision>"""
    )

    # PATCH 9 – Remove PosePublisher plugin (conflicts with static TF)
    sdf = sdf.replace(
        """    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <publish_nested_model_pose>true</publish_nested_model_pose>
    </plugin>""",
        """    <!-- PosePublisher removed: conflicted with static_transform_publisher -->"""
    )

    # =========================================================================
    # LAUNCH NODES
    # =========================================================================

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r {patched_world_path}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', sdf,
            '-name', 'robot',
            '-x', '-1.0', '-y', '-0.5', '-z', '0.15'
        ],
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
        remappings=[
            ('/cmd_vel', '/robot/cmd_vel'),
            ('/odom',    '/robot/odom'),
            ('/scan',    '/robot/scan'),
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

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': 'true'
        }.items()
    )

    slam_delayed = TimerAction(period=5.0, actions=[slam_toolbox_launch])

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        parameter_bridge,
        lidar_static_tf,
        slam_delayed,
    ])
