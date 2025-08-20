import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_vtvl_sim = get_package_share_directory('sim')

    # Path to the world file
    world_file = os.path.join(pkg_vtvl_sim, 'worlds', 'basic_pad.world')

    # Launch Gazebo with the world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Path to the SDF model file
    sdf_file = os.path.join(pkg_vtvl_sim, 'models', 'simple_rocket', 'model.sdf')

    # Spawn the rocket entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'vtvl_rocket', '-file', sdf_file],
        output='screen'
    )

    # Launch your C++ nodes
    sim_dynamics_node = Node(
        package='sim',
        executable='sim_dynamics',
        name='sim_dynamics_node',
        output='screen'
    )

    planar_controller_node = Node(
        package='sim',
        executable='planar_controller',
        name='planar_controller_node',
        output='screen',
        parameters=[{'setpoint.altitude': 50.0}]
    )

    # Node to update Gazebo model pose from our simulation
    visualizer_node = Node(
        package='sim',
        executable='visualizer_node.py', # The name of our python script
        name='visualizer_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        sim_dynamics_node,
        planar_controller_node,
        visualizer_node,
    ])