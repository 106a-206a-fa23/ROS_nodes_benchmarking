# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='performance_monitor',
#             executable='monitor',
#             name='monitor',
#         )
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
# from base_common import get_param_file

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('performance_monitor'),
        'param',
        'performance_monitor.param.yaml'
    )

    params_arg = DeclareLaunchArgument(
        'performance_monitor_params',
        default_value=params_file,
        description='Path to params yaml'
    )

    # params = get_param_file("performance_monitor")

    # Get the path to the Python script
    # node_file = os.path.join(
    #     get_package_share_directory('performance_monitor'),
    #     'test_performance.py'
    # )
    node = Node(
        package='performance_monitor',
        executable='perf_monitor',
        name='perf_monitor',
        parameters=[params_file],
        )

    # Declare launch arguments
    # my_arg = DeclareLaunchArgument(
    #     'my_arg',
    #     default_value='some_default_value',
    #     description='An example launch argument for my_python_node.py'
    # )

    # Launch the Python node
    # node_action = ExecuteProcess(
    #     cmd=['python3', node_file],#, '--my-arg', LaunchConfiguration('my_arg')],
    #     output='screen',
    # )

    # Create the launch description with the defined actions
    ld = LaunchDescription()
    # ld.add_action(my_arg)
    ld.add_action(params_arg)
    ld.add_action(node)

    return ld
