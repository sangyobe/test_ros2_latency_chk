import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
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
                        {'use_intra_process_comms' : True }
                    ])
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])