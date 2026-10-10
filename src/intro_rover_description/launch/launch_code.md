# run ros2 file
cd /home/ahmet/projects/Software_Intro_Task_2026-2027
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch intro_rover_description view_rover.launch.py

# manual rviz
ros2 launch intro_rover_description view_rover.launch.py
# run animation
ros2 launch intro_rover_description dance.launch.py

# GUI
cd /home/ahmet/projects/Software_Intro_Task_2026-2027
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./src/intro_rover_description/scripts/run_rover_gui.sh
