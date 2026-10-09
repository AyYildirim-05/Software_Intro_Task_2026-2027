# run ros2 file
cd /home/ahmet/projects/Software_Intro_Task_2026-2027
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch intro_rover_description view_rover.launch.py

# run saved rviz file
cd /home/ahmet/projects/Software_Intro_Task_2026-2027
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rviz2 -d src/intro_rover_description/config/rover.rviz

# run animation
ros2 launch intro_rover_description dance.launch.py
ros2 topic echo /joint_states
ros2 topic echo /tf


# GUI
cd /home/ahmet/projects/Software_Intro_Task_2026-2027
./src/intro_rover_description/scripts/run_rover_gui.sh

## terminal a
cd /home/ahmet/projects/Software_Intro_Task_2026-2027
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/ahmet/projects/Software_Intro_Task_2026-2027/install/intro_rover_description/share:${GZ_SIM_RESOURCE_PATH:-}
gz sim /usr/share/gz/gz-sim/worlds/empty.sdf -s -r -v 4

## terminal b
cd /home/ahmet/projects/Software_Intro_Task_2026-2027
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/ahmet/projects/Software_Intro_Task_2026-2027/install/intro_rover_description/share:${GZ_SIM_RESOURCE_PATH:-}
ros2 launch intro_rover_description spawn_gazebo.launch.py

## terminal c
ros2 run intro_rover_description jointstate.py

## terminal d
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once