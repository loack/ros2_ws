from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_kuka_minus',
            output='screen',
            parameters=[{'robot_description': open('urdf/kuka_minus.urdf').read()}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_kuka_cortex',
            output='screen',
            parameters=[{'robot_description': open('urdf/kuka_cortex.urdf').read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/path/to/your/rviz/config/file.rviz']  # Update with your RViz config path
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=['config/gamepad.yaml']
        )
    ])