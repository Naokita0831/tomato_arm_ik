from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('tomato_arm_ik')
    model_share = FindPackageShare('crane_plus_description')

    # model_arg = DeclareLaunchArgument(
    #     'model',
    #     default_value=PathJoinSubstitution([pkg_share, 'urdf', 'crane_plus.urdf.xacro']),
    #     description='URDF or xacro file path',
    # )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            pkg_share,
            'urdf',
            'crane_plus.urdf.xacro',
        ]),
        description='URDF or xacro file path',
    )

    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyUSB0',
        description='Dynamixel USB device path',
    )

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            LaunchConfiguration('model'),
            ' ',
            'use_gazebo:=false',
            ' ',
            'use_camera:=false',
            ' ',
            'use_mock_components:=true',
        ])
    }

    return LaunchDescription([
        model_arg,
        dev_arg,
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='tomato_arm_ik',
            executable='arm_ik_joy',
            name='arm_ik_joy',
            output='screen',
            parameters=[{
                'timer_period_ms': 50,
                'trajectory_time_sec': 0.05,
                'publish_joint_states': True,
            }],
        ),
        Node(
            package='tomato_arm_ik',
            executable='crane_control',
            name='crane_control',
            output='screen',
            parameters=[{
                'dev': LaunchConfiguration('dev'),
                'source_topic': 'joint_states',
                'use_sync_write': True,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'arm_display.rviz'])],
            output='screen',
        ),
    ])
