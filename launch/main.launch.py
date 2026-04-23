from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    aiil_demo_dir = get_package_share_directory('aiil_rosbot_demo')
    find_object_launch = os.path.join(
        aiil_demo_dir,
        'launch',
        'find_object_2d.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'objects_path',
            default_value='~/aiil_workspace/humble_workspace/src/par_coursework/objects_example',
            description='Path to trained hazard marker images'
        ),

        DeclareLaunchArgument(
            'find_object_gui',
            default_value='false',
            description='Open find_object_2d GUI'
        ),

        DeclareLaunchArgument(
            'enable_hazard_detector',
            default_value='true',
            description='Start hazard_detector bridge node'
        ),

        DeclareLaunchArgument(
            'enable_hazard_detection',
            default_value='true',
            description='Start hazard_detection_node mapper node'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(find_object_launch),
            launch_arguments={
                'gui': LaunchConfiguration('find_object_gui'),
                'objects_path': LaunchConfiguration('objects_path'),
            }.items()
        ),

        Node(
            package='par_snc',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),

        Node(
            package='par_snc',
            executable='hazard_detector',
            name='hazard_detector_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_hazard_detector'))
        ),

        Node(
            package='par_snc',
            executable='hazard_detection_node',
            name='hazard_detection_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_hazard_detection'))
        ),

        Node(
            package='par_snc',
            executable='path_node',
            name='path_node',
            output='screen'
        ),
    ])