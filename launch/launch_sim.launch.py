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
    
    param_file_name =  'snail.yaml'
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]\
                  
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(get_package_share_directory('snail_bot'),'param',param_file_name))
    
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

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

    tele_joy = IncludeLaunchDescription(         #telejoy "node"
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]),launch_arguments={'use_sim_time': use_sim_time}.items()
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

    # amcl = Node(
    #     parameters=[
    #       {'use_sim_time': use_sim_time},
    #       {'map':  map_dir}
    #     ],
    #     package='nav2_brirngup',
    #     executable='localization.launch.py',
    #     name='amcl',
    #     output='screen')

    

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    
    
    

    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)
    ld.add_action(tele_joy)


    
    
    

    return ld

