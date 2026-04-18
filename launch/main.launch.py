from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'demo_object_name',
            default_value='explosive'
        ),

        Node(
            package='par_snc',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),

        Node(
            package='par_snc',
            executable='hazard_node',
            name='hazard_node',
            output='screen',
            parameters=[
                {'demo_object_name': LaunchConfiguration('demo_object_name')}
            ]
        ),

        Node(
            package='par_snc',
            executable='path_node',
            name='path_node',
            output='screen'
        ),
    ])