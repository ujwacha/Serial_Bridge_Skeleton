import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    "generate launch descriptions for the composable node"

    container = ComposableNodeContainer(
        name='tester_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='Serial_Bridge_Skeleton',
                plugin='Serial_Bridge_Skeleton::Sayer',
                name='thingsayer',
                # ..
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    return launch.LaunchDescription([container])

    
