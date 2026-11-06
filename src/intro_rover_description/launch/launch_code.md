# RViz Visualization
ros2 launch intro_rover_description view_rover.launch.py

# RViz Animation / Dance
ros2 launch intro_rover_description dance.launch.py


# build
cd ~/projects/Software_Intro_Task_2026-2027
colcon build --packages-select intro_rover_description --symlink-install

# launch
source install/setup.bash
ros2 launch intro_rover_description gazebo_rover.launch.py

## tele operatpr
ros2 control list_controllers

cd ~/projects/Software_Intro_Task_2026-2027
source install/setup.bash
ros2 run intro_rover_description rover_teleop.py

## PS4
ros2 launch intro_rover_description ps4_teleop.launch.py
