import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('test_robot_description')
    world_path = os.path.join(pkg_dir, 'worlds', 'test_ground.sdf')
    robot_sdf_path = os.path.join(pkg_dir, 'models', 'custom_robot.sdf')

    with open(robot_sdf_path, 'r') as f:
        sdf = f.read()

    # =========================================================================
    # RUNTIME SDF PATCHES
    # =========================================================================

    # PATCH 1 – self_collide off
    sdf = sdf.replace(
        '<model name="diff_drive_robot">',
        '<model name="diff_drive_robot">\n    <self_collide>false</self_collide>'
    )

    # PATCH 2 – Fix wheel joint axes + add damping
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

    # PATCH 3 – Left wheel: inertia + corrected z-pose + friction
    sdf = sdf.replace(
        """<link name="left_wheel">
      <pose>0.15 0.225 -0.025 -1.5707 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
      </collision>""",
        """<link name="left_wheel">
      <pose>0.15 0.225 0.0 -1.5707 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001302</ixx><iyy>0.001302</iyy><izz>0.0025</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>0.3</mu2></ode></friction>
          <contact><ode><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>"""
    )

    # PATCH 4 – Right wheel: inertia + corrected z-pose + friction
    sdf = sdf.replace(
        """<link name="right_wheel">
      <pose>0.15 -0.225 -0.025 -1.5707 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
      </collision>""",
        """<link name="right_wheel">
      <pose>0.15 -0.225 0.0 -1.5707 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001302</ixx><iyy>0.001302</iyy><izz>0.0025</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>0.3</mu2></ode></friction>
          <contact><ode><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>"""
    )

    # PATCH 5 – Caster: inertia + corrected z-pose + low friction
    sdf = sdf.replace(
        """<link name="caster">
      <pose>-0.2 0 -0.08 0 0 0</pose>
      <collision name="collision">
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
      </collision>""",
        """<link name="caster">
      <pose>-0.2 0 -0.05 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0.01</mu><mu2>0.01</mu2></ode></friction>
          <contact><ode><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>"""
    )

    # PATCH 6 – Caster joint: ball → fixed
    sdf = sdf.replace(
        '<joint name="caster_joint" type="ball">',
        '<joint name="caster_joint" type="fixed">'
    )

    # PATCH 7 – Inject <topic> into both sensors
    #
    # Without <topic>, Gz Harmonic auto-publishes on a long scoped path:
    #   /world/test_sensor_world/model/test_robot/link/lidar_link/sensor/gpu_lidar/scan
    # The bridge listens on /scan — that auto-path didn't match, so nothing bridged.
    #
    # Do NOT inject <frame_id> here: Gz Harmonic ignores <frame_id> as a direct
    # <sensor> child and always sets the LaserScan header.frame_id to the scoped
    # link name: 'test_robot/lidar_link' (model_name/link_name).
    # The static TF publishers below use that exact string as their child-frame-id.
    sdf = sdf.replace(
        '<sensor name="gpu_lidar" type="gpu_lidar">',
        '<sensor name="gpu_lidar" type="gpu_lidar">\n        <topic>/scan</topic>'
    )
    sdf = sdf.replace(
        '<sensor name="front_camera" type="camera">',
        '<sensor name="front_camera" type="camera">\n        <topic>/camera</topic>'
    )

    # =========================================================================
    # LAUNCH ACTIONS
    # =========================================================================

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', sdf,
            '-name', 'test_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.10'
        ],
        output='screen'
    )

    # Topic bridge
    #
    # The DiffDrive plugin writes frame_id='odom' and child_frame_id='base_link'
    # LITERALLY into the TF/Odometry messages — it does NOT prefix the model
    # name.  So the TF tree that arrives in ROS is:
    #
    #   odom → base_link       (plain, no "test_robot/" prefix)
    #
    # The static TF publishers below must use those same plain names so the
    # chain is connected.  RViz Fixed Frame must also be 'odom' (plain).
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='multi_sensor_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/scan',        '/test_robot/scan'),
            ('/camera',      '/test_robot/camera/image_raw'),
            ('/camera_info', '/test_robot/camera/camera_info'),
            ('/odom',        '/test_robot/odom'),
            ('/cmd_vel',     '/test_robot/cmd_vel'),
        ],
        output='screen'
    )

    # Static TF: base_link → test_robot/lidar_link/gpu_lidar
    #
    # Gz Harmonic sets LaserScan.header.frame_id to the FULL sensor-scoped name:
    # model/link/sensor = 'test_robot/lidar_link/gpu_lidar'  (confirmed by:
    #   ros2 topic echo /test_robot/scan --once | grep frame_id)
    # child-frame-id must match this exactly — not just the link name.
    # No use_sim_time — static transforms are valid for ALL time by definition.
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_publisher',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.1',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'test_robot/lidar_link/gpu_lidar'
        ],
        output='screen'
    )

    # Static TF: base_link → test_robot/camera_link
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf_publisher',
        arguments=[
            '--x', '0.25', '--y', '0.0', '--z', '0.02',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'test_robot/camera_link'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        parameter_bridge,
        lidar_static_tf,
        camera_static_tf,
    ])
