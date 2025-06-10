from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_system',
            executable='perception_system_node',
            # name='perception_system',
            output='screen',
            parameters=[{
                'test': True,
                # 'images_path': '/home/atu-2/robothon/src/perception_system/images/',
                'weight_path': '/home/atu-2/robothon/src/perception_system/weights/train/weights/best.pt',
                'labels_path': '/home/atu-2/robothon/src/perception_system/weights/dataset.yaml'
            }],
        )
    ])
