import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container_server = ComposableNodeContainer(
            name='latency_chk_server_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='latency_chk',
                    plugin='latency_chk::SystemHealthCheckServerImpl',
                    name='latency_check_server',
                    extra_arguments=[
                        {'use_intra_process_comms' : False }
                    ]
                )
            ],
            output='screen',
    )
    
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
                        {'runs': 5000, 'size': 1*1024, 'log_file' : 'latency_roundtrip.log'}
                    ],
                    remappings=[
                        ('/latency_chk_payload', '/latency_chk_payload_echo')
                    ]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([
        container_server, 
        container_client
    ])