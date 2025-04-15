from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    demo_cmd = Node(
        package='opennav_coverage_demo',
        executable='demo_row_coverage_maize',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        demo_cmd
    ])
