import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_vtvl_sim = get_package_share_directory('sim')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Spawn rocket
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'vtvl_rocket'],
                        output='screen')

    # Static TF publisher
    # This is a workaround to publish the model to the /robot_description topic
    static_tf_pub = Node(package='tf2_ros', executable='static_transform_publisher',
                         name='static_transform_publisher_node',
                         arguments=['0', '0', '0', '0', '0', '0', 'world', 'vtvl_rocket/base_link'],
                         output='screen')
    
    # Our custom nodes
    sim_dynamics_node = Node(package='sim', executable='sim_dynamics',
                             name='sim_dynamics_node',
                             output='screen')
    
    planar_controller_node = Node(package='sim', executable='planar_controller',
                                 name='planar_controller_node',
                                 output='screen',
                                 parameters=[{'setpoint.altitude': 50.0}])

    # Gazebo doesn't directly display our custom state. We need a bridge.
    # For now, we will skip this and add it if needed. The core issue can be debugged without it.
    # The simplest way is to have Gazebo run the physics, but for now we want to see our own physics.

    return LaunchDescription([
        gazebo,
        static_tf_pub, # This is a trick to get the model description published
        spawn_entity,
        sim_dynamics_node,
        planar_controller_node,
    ])