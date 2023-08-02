import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    container_client = ComposableNodeContainer(
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
                        {'use_intra_process_comms' : False }
                    ],
                    parameters=[
                        {'runs': 5000, 'size': -1, 'log_file' : 'latency_client.log'}
                    ]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container_client])