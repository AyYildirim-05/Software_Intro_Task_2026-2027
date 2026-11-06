# Repository Description
This is the intro task repository to be forked by new potential software members who are interested in contributing to the software of the Robotics division of Space Concordia.\
The instructions of the software introduction task can be found in this README.md below.\
This repository contains a ROS2 workspace where packages can be added inside of the /src folder.\
Keep in mind that the URDF is very detailed and might need to be modified in order to reduce detail.\
Remember to go over documentation.


## 3. Visualize the robot in RViz
3.1. Create a launch file to visualize the rover in the RViz software.

## 4. Create a Ros2 infrastructure to control the rover in RViz
4.1. Create a node to manually control each joint of the rover.
4.2. Create a node to make the rover dance. This should be a reprogrammed sequence or a function of the joint states over time.

## 5. Simulate the robot in Gazebo Harmonic
5.1. Create a launch file that spawns the rover in a Gazebo world. Please make sure to use Gazebo Harmonic as this version of Gazebo works better with ROS2 Jazzy.
5.2. Control the rover with the control infrastructure built in part 4. and see it move in the Gazebo world.

## 6. Be ready to present your solution
6.1. Modify this README.md file in your forked workspace presenting your solutions.\
6.2. Run and present your workspace to the Software Manager.

---

## PS4 Controller Teleoperation & Mapping Guide

### Default Control Layout
* **Left Stick (`axes[1]` Up/Down, `axes[0]` Left/Right):** Drives the rover (Forward/Backward, 4-Wheel Ackermann turning, and in-place swerve spinning).
* **Arm Joint Selection Buttons (Chronological from Base $\rightarrow$ End-Effector):**
  1. `buttons[0]` (**Cross / X**) $\rightarrow$ `shoulder_yaw`
  2. `buttons[1]` (**Circle / O**) $\rightarrow$ `shoulder_pitch`
  3. `buttons[2]` (**Triangle**) $\rightarrow$ `elbow_pitch`
  4. `buttons[3]` (**Square**) $\rightarrow$ `elbow_roll`
  5. `buttons[4]` (**L1**) $\rightarrow$ `wrist_pitch`
  6. `buttons[5]` (**R1**) $\rightarrow$ `wrist_roll`
  * `buttons[9]` (**Options**) $\rightarrow$ Reset all arm joints to Home (`0.0` rad)
* **Right Stick (`axes[3]` / `axes[4]`):** Moves the selected arm joint (`+` positive = clockwise, `-` negative = counter-clockwise).

### How to Check & Remap Controller Indices
If your controller uses different button or stick indices:

#### Step 1: Find out which index is which
Plug in the controller and run these two commands in separate terminals:
```bash
# Terminal 1: Start the joystick driver
ros2 run joy joy_node

# Terminal 2: Watch the live /joy messages
ros2 topic echo /joy
```
You will see live `axes:` and `buttons:` arrays (both 0-indexed):
```yaml
axes:
- 0.0    # Index 0
- 0.0    # Index 1
- 1.0    # Index 2
- 0.0    # Index 3
- 0.0    # Index 4
- 1.0    # Index 5
buttons:
- 0      # Index 0
- 0      # Index 1
- 0      # Index 2
- 0      # Index 3
- 0      # Index 4
- 0      # Index 5
```
* **Press a button:** Whichever slot in `buttons:` flips from `0` to `1` is that button's index.
* **Push a stick:** Whichever slot in `axes:` changes from `0.0` toward `1.0` or `-1.0` is that stick axis's index.

#### Step 2: Modify `src/intro_rover_description/config/ps4_controller.yaml`
Update the indices in `src/intro_rover_description/config/ps4_controller.yaml` (no Python code changes required):
```yaml
joy_demux_node:
  ros__parameters:
    deadzone: 0.12

    # 1. Left Stick Up/Down and Left/Right axis indices:
    drive_forward_axis: 1             # <-- Index that moves when pushing Left Stick Up/Down
    drive_turn_axis: 0                # <-- Index that moves when pushing Left Stick Left/Right
    max_wheel_speed: 8.0

    # 2. Right Stick Left/Right and Up/Down axis indices:
    arm_stick_horizontal_axis: 3      # <-- Right Stick Left/Right index
    arm_stick_vertical_axis: 4        # <-- Right Stick Up/Down index
    arm_joint_speed: 1.5

    # 3. 6 Button indices for the 6 arm joints in chronological order:
    # [shoulder_yaw, shoulder_pitch, elbow_pitch, elbow_roll, wrist_pitch, wrist_roll]
    arm_joint_buttons: [0, 1, 2, 3, 4, 5]
    arm_reset_button: 9
```
Then rebuild and source the workspace:
```bash
colcon build --packages-select intro_rover_description
source install/setup.bash
```
