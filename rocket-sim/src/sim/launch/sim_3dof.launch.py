import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_vtvl_sim = get_package_share_directory('sim')

    # Define the world file
    world_file = os.path.join(pkg_vtvl_sim, 'worlds', 'basic_pad.world')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        # THIS IS THE CRUCIAL PART
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        }.items(),
    )

    # Get the SDF file path
    sdf_file = os.path.join(pkg_vtvl_sim, 'models', 'simple_rocket', 'model.sdf')

    # Spawn rocket entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'vtvl_rocket', '-file', sdf_file],
                        output='screen')

    # Our custom nodes
    sim_dynamics_node = Node(package='sim', executable='sim_dynamics',
                             name='sim_dynamics_node',
                             output='screen')

    planar_controller_node = Node(package='sim', executable='planar_controller',
                                 name='planar_controller_node',
                                 output='screen',
                                 parameters=[{'setpoint.altitude': 50.0}])

    return LaunchDescription([
        gazebo,
        spawn_entity,
        sim_dynamics_node,
        planar_controller_node,
    ])