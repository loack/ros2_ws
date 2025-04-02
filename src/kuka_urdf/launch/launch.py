import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #Remus robot
    urdf_file_name = 'kr210l150_remus.urdf'
    urdf_1 = os.path.join(
        get_package_share_directory('kuka_urdf'),'urdf',
        urdf_file_name)
    with open(urdf_1, 'r') as infp:
        robot_desc_1 = infp.read()

    #romulus robot
    urdf_file_name = 'kr210l150_romulus.urdf'
    urdf_2 = os.path.join(
        get_package_share_directory('kuka_urdf'),'urdf',
        urdf_file_name)
    with open(urdf_2, 'r') as infp:
        robot_desc_2 = infp.read()

    rviz_config_path = os.path.join(get_package_share_directory('kuka_urdf'), 'urdf', 'kuka.rviz')
    if not os.path.exists(rviz_config_path):
        raise FileNotFoundError(f"RViz config file not found: {rviz_config_path}")
    rviz_config = rviz_config_path
    print(f"Using RViz config: {rviz_config}")
        
    return LaunchDescription([
        #RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        #Remus robot state publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='remus_state_publisher',
            namespace='remus',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf_1])}], #Command(['xacro ', urdf])
            arguments=[urdf_1]),

        #Romulus robot state publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='romulus_state_publisher',
            namespace='romulus',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf_2])}], #Command(['xacro ', urdf])
            arguments=[urdf_2]),
        
        # Static transform for Remus robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='remus_static_tf',
            namespace='remus',
            arguments=['0', '-1.5', '0', '0', '0', '0', 'world', 'remus_base_link']
        ),

        # Static transform for Romulus robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='romulus_static_tf',
            arguments=['0', '1.5', '0', '0', '0', '0', 'world', 'romulus_base_link']
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='remus_joint_state_publisher',
            namespace='remus',
            output='screen',
            #remappings=[('/joint_states', '/remus/joint_states')]
        ),
        #joint state publisher GUI
        Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='remus_joint_state_publisher_gui',
        namespace='remus',
        output='screen',
        #remappings=[('/joint_states', '/remus/joint_states')]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='romulus_joint_state_publisher',
            namespace='romulus',
            output='screen',
            #remappings=[('/joint_states', '/romulus/joint_states')]
        ),
        
        #joint state publisher GUI
        Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='romulus_joint_state_publisher_gui',
        namespace='romulus',
        output='screen',
        #remappings=[('/joint_states', '/romulus/joint_states')]
        ),
    ])