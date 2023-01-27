from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('snail_bot'),'config','joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory('snail_bot'),'config','twist_mux.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params,
                         {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, 
                        {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )
    
    twist_mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', 'diff_cont/cmd_vel_unstamped')},
            parameters=[twist_mux_params
                        #{'use_sim_time': use_sim_time}
                        ],
        )

    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')]
    #      )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)

    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(twist_mux_node)
    
    #ld.add_action(twist_mux_node)

    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value='false',
    #         description='Use sim time if true'),
    #     joy_node,
    #     teleop_node,
    #     # twist_stamper       
    # ])

    return ld