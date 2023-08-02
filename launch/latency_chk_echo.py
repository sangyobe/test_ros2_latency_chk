from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='latency_chk',
            namespace='/',
            executable='latency_chk_echo',
            name='latency_chk_echo'
        )
    ])