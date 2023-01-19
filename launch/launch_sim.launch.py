import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='snail_bot' #<--- CHANGE ME

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    param_file_name =  'snail.yaml'
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join('/home/engineerbear/Documents/prac_ws/src/snail_bot/param/',param_file_name))

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join('/home/engineerbear/Documents/prac_ws/src/snail_bot/worlds/','map_save.yaml'))

    declare_map =  DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')


    rsp = IncludeLaunchDescription(         #robot state publisher "node"
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'snail_bot'],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # map_server = Node(
    #     parameters=[
    #       {'yaml_filename': map_file},
    #       {'use_sim_time': use_sim_time}
    #     ],
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen')

    # nav2_lifecycle_map_mangr = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_mapper',
    #     output='screen',
    #     parameters=[
    #       {'autostart': True},
    #       {'use_sim_time': use_sim_time},
    #       {'node_names': ['map_server']}  
    #     ])

    amcl = Node(
        parameters=[
          {'use_sim_time': use_sim_time},
          {'map':  map_dir}
        ],
        package='nav2_brirngup',
        executable='localization.launch.py',
        name='amcl',
        output='screen')

    # nav2_lifecycle_amcl_mangr = Node(
    #     package='nav2_util',
    #     executable='life',
    #     name='lifecycle_manager_amcl',
    #     output='screen',
    #     parameters=[
    #       {'autostart': True},
    #       {'use_sim_time': use_sim_time},
    #       {'node_names': ['amcl']}  
    #     ])
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    navstack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)
    #ld.add_action(navstack)
    # ld.add_action(nav2_lifecycle_map_mangr)
    # ld.add_action(map_server)
    # ld.add_action(nav2_lifecycle_amcl_mangr)
    #ld.add_action(amcl)
    #ld.add_action(start_async_slam_toolbox_node)

    return ld

