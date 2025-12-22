from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        # Robot State Publisher with Xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        PathJoinSubstitution([FindExecutable(name='xacro')]),
                        ' ',
                        PathJoinSubstitution([
                            FindPackageShare('primo-arm-r01'),
                            'urdf',
                            'primio-arm-r01.urdf.xacro'
                        ])
                    ]),
                    value_type=str
                )
            }]
        ),

        # Joint State Publisher GUI
        #Node(
        #    package='joint_state_publisher_gui',
        #    executable='joint_state_publisher_gui',
        #    name='joint_state_publisher_gui',
        #    output='screen'
        #),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

