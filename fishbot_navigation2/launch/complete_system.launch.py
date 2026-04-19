# import os
# import launch
# import launch_ros
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():
#     # 获取功能包路径
#     fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2') 
#     fishbot_description_dir = get_package_share_directory('../share/fishbot_description')
    
#     # 控制是否启动导航
#     launch_nav2 = launch.substitutions.LaunchConfiguration('launch_nav2', default='true')

#     return launch.LaunchDescription([
#         # 声明新的 Launch 参数
#         launch.actions.DeclareLaunchArgument(
#             'launch_nav2', 
#             default_value=launch_nav2, 
#             description='Whether to launch navigation'
#         ),

#         # 启动gazebo_sim.launch.py
#         launch.actions.IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([fishbot_description_dir, '/launch', '/gazebo_sim.launch.py']),
#         ),
        
#         # 启动navigation.launch.py
#          launch.actions.IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([fishbot_navigation2_dir, '/launch', '/navigation.launch.py']),
#         ),
#     ])