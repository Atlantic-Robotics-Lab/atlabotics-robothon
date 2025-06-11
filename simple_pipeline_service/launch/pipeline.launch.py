import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    # Declare the launch arguments
    moondream_api_key_arg = DeclareLaunchArgument(
        'moondream_api_key',
        default_value=EnvironmentVariable('MOONDREAM_API_KEY', default_value=''),
        description='API key for the Moondream service.'
    )

    google_api_key_arg = DeclareLaunchArgument(
        'google_api_key',
        default_value=EnvironmentVariable('GOOGLE_API_KEY', default_value=''),
        description='API key for the Google Gemini service.'
    )

    # Define the node that will be launched
    pipeline_node = Node(
        package='simple_pipeline_service',           
        executable='pipeline_node',
        name='pipeline_service_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # Pass the launch configurations to the node as parameters
            'moondream_api_key': LaunchConfiguration('moondream_api_key'),
            'google_api_key': LaunchConfiguration('google_api_key'),
        }]
    )

    # Create the launch description and add the actions
    ld = LaunchDescription()
    
    ld.add_action(moondream_api_key_arg)
    ld.add_action(google_api_key_arg)
    ld.add_action(pipeline_node)

    return ld
