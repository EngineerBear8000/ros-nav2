ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

for keyboard control

ros2 launch snail_bot launch_sim.launch.py world:=./src/snail_bot/worlds/playfield.world

run launch file for gazebo, rsp, lidar and ros_control stuff

ros2 launch slam_toolbox online_async_launch.py params_file:=./src/snail_bot/config/mapper_params_online_async.yaml use_sim_time:=true

launch slam

install ros-foxy-ros2-control ros-foxy-ros2-controllers slam-toolbox

might need ros-foxy-gazebo-ros2-control

sudo apt install ros-humble-twist-mux
need twist mux 

ros2 run twist_mux twist_mux --ros-args --params-file ./src/snail_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

run twist mux 

ros2 launch nav2_bringup localization_launch.py map:=/home/engineerbear/Documents/prac_ws/src/snail_bot/worlds/map_save.yaml use_sim_time:=true

run localisation

ros2 launch nav2_bringup navigation_launch.py 
run nav

ros2 run tf2_tools view_frames
capture frames