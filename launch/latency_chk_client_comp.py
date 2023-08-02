import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    runs = LaunchConfiguration('runs')
    runs_launch_arg = DeclareLaunchArgument(
        'runs',
        default_value='1'
    )
    
    container = ComposableNodeContainer(
            name='latency_chk_client_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='latency_chk',
                    plugin='latency_chk::SystemHealthCheckClientImpl',
                    name='latency_check_client',
                    extra_arguments=[
                        {'use_intra_process_comms' : True }
                    ],
                    parameters=[
                        {'runs': 5000, 'size': 1*1024, 'log_file' : 'latency.log'}
                    ])
            ],
            output='screen',
    )

    return launch.LaunchDescription([container, runs_launch_arg])