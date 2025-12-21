import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sim = get_package_share_directory('bme_gazebo_sensors')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'spawn_robot.launch.py')
        ),
    )

    ui = Node(
        package='assignment2_rt',
        executable='ui',
        name='UI_node',
        output='screen'
    )

    safety = Node(
        package='assignment2_rt',
        executable='safety',
        name='Safety_node',
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        safety,
        ui,
    ])
