
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    param_file_name =  'snail.yaml'
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    #config declarations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(get_package_share_directory('snail_bot'),'worlds/','map_save.yaml'))
    
    amcl_yaml_dir = LaunchConfiguration(
        'amcl_yaml_0',
        default=os.path.join(get_package_share_directory('snail_bot'),'config','map_save.yaml'))
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(get_package_share_directory('snail_bot'),'param',param_file_name))
    

    #arg declarations
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_map =  DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load')

    declare_amcl = DeclareLaunchArgument(
        'amcl_yaml_0',
        default_value='amcl_yaml_dir',
        description='Full path to amcl file to load')

    navstack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
    )

    map_server = Node(
        parameters=[
          {'yaml_filename': map_dir},
          {'use_sim_time': use_sim_time}
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen')

    amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
            amcl_yaml_dir,
            {'use_sim_time': use_sim_time}
            ])

    

    nav2_lifecycle_mangr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
        {'autostart': True},
        {'use_sim_time': use_sim_time},
        {'bond_timeout':0.0},
        {'node_names': ['map_server']}  
        ])

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    #ld.add_action(declare_amcl)

    #ld.add_action(amcl)
    ld.add_action(nav2_lifecycle_mangr)
    ld.add_action(map_server)

    ld.add_action(navstack)

    return ld