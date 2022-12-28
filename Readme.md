ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

for keyboard control

ros2 launch snail_bot launch_sim.launch.py world:=./src/snail_bot/worlds/playfield.world

run launch file for gazebo, rsp, lidar and ros_control stuff

install ros-foxy-ros2-control ros-foxy-ros2-controllers slam-toolbox

might need ros-foxy-gazebo-ros2-control