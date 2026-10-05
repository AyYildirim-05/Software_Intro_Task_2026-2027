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