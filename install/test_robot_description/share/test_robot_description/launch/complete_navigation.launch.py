import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # <── We imported Node to run RViz directly

def generate_launch_description():
    # 1. Locate the packages on the system
    my_package_dir = get_package_share_directory('test_robot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 2. Define the exact paths to map and parameters files
    map_yaml_file = '/home/s43/test_sensor_ws/src/test_robot_description/maps/my_perfect_room_map.yaml'
    nav2_params_file = os.path.join(my_package_dir, 'config', 'nav2_params.yaml')

    # 3. Include Gazebo Simulation launch sequence
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_package_dir, 'launch', 'run_experiment_launch.py')
        )
    )

    # 4. Calling the official Nav2 bringup launch file (without the ignored use_rviz argument)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    # 5. Launch RViz explicitly using the Nav2 default configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 6. Return the combined launch sequence
    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        rviz_node   # <── Added RViz to the execution list!
    ])
