import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    autopatrol_robot_dir = get_package_share_directory('autopatrol_robot_cpp')
    patrol_config_path = os.path.join(autopatrol_robot_dir, 'config', 'patrol_config.yaml')
    
    # 启动巡检节点
    action_patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot_cpp',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
        parameters=[patrol_config_path]
    )
    
    action_speaker_node = launch_ros.actions.Node(
        package='autopatrol_robot_cpp',
        executable='speaker',
        name='speaker',
        output='screen'
    )

    return launch.LaunchDescription([
        action_speaker_node, 
        action_patrol_node,
    ])