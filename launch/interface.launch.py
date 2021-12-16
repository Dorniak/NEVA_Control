import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/control.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        Node(
            package='NEVA_control',
            executable='interfaz',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        )
    ])
