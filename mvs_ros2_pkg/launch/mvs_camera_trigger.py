from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('mvs_ros2_driver')

    # 声明一个布尔参数 show_rviz，默认 True
    declare_show_rviz_arg = DeclareLaunchArgument(
        'show_rviz',
        default_value='false',
        description='Whether to launch RViz2'
    )

    def launch_setup(context, *args, **kwargs):
        show_rviz = LaunchConfiguration('show_rviz').perform(context)

        # 相机节点
        camera_node = Node(
            package='mvs_ros2_driver',
            executable='mvs_camera_node',
            name='mvs_camera_trigger',
            arguments=[os.path.join(pkg_dir, 'config/left_camera_trigger.yaml'),
                       '--ros-args', '--log-level', 'info'],
            respawn=True,
            output='screen',
        )

        nodes_to_launch = [camera_node]

        # 根据布尔参数决定是否启动 RViz
        if show_rviz.lower() in ['true', '1', 'yes']:
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_dir, 'rviz/mvs_camera.rviz')],
                output='screen'
            )
            nodes_to_launch.append(rviz_node)

        return nodes_to_launch

    return LaunchDescription([
        declare_show_rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])