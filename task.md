# Diagnostic Report & Fix Plan: Wheel Velocity Controller Error

## 1. Problem Summary
When streaming commands from the custom ROS 2 teleop node, Gazebo throws continuous validation errors:
[gazebo-1] [ERROR] [wheel_velocity_controller]: Mismatch between joint_names size (4) and positions (0) at point #0.

As a result, incoming trajectory commands are discarded, and the rover's wheels never generate drive torque or rotation despite the steering/swerve joints operating normally.

---

## 2. Root Cause Analysis
The teleop node transmits `trajectory_msgs/msg/JointTrajectory` packets containing only `velocities = [...]` while leaving `positions` unassigned (size 0). 

In `ros2_control`:
1. If `wheel_velocity_controller` is configured as a `joint_trajectory_controller/JointTrajectoryController` whose `command_interfaces` includes `position`, the controller enforces that `len(point.positions) == len(joint_names)`. An empty array causes validation to fail immediately.
2. Alternatively, if `wheel_velocity_controller` was intended to be a velocity controller, using `JointTrajectory` is invalid if the controller plugin type is `velocity_controllers/JointGroupVelocityController` (which expects `std_msgs/msg/Float64MultiArray` on the topic `/wheel_velocity_controller/commands`).

---

## 3. Resolution Steps

### Step A: Verify Controller Type & Configuration
Inspect the active controller configuration in your package's `ros2_controllers.yaml`:

```bash
# Check the controller type and active state:
ros2 control list_controllers
Scenario 1: If using joint_trajectory_controller/JointTrajectoryController
Ensure the parameter block in ros2_controllers.yaml is configured strictly for open-loop velocity control so it does not demand joint positions:

YAML
wheel_velocity_controller:
  ros__parameters:
    joints:
      - fl_wheel
      - fr_wheel
      - bl_wheel
      - br_wheel
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity
    open_loop_control: true
    allow_integration_in_goal_trajectories: false
Scenario 2: If using velocity_controllers/JointGroupVelocityController (Standard for Wheels)
If the configuration defines:

YAML
wheel_velocity_controller:
  ros__parameters:
    type: velocity_controllers/JointGroupVelocityController
Then the controller does not accept JointTrajectory messages. The teleop node must publish std_msgs/msg/Float64MultiArray to /wheel_velocity_controller/commands.

4. Teleop Node Code Updates
Depending on your chosen architecture above, implement the matching patch in the teleop script:

Patch 1: For JointTrajectoryController (Sending dummy positions or pure velocity goals)
If keeping JointTrajectory, ensure positions matches the joint count if command_interfaces cannot be changed:

Python
# Inside RoverTeleopNode.timer_callback:
traj_w = JointTrajectory()
traj_w.joint_names = self.wheel_joints
pt_w = JointTrajectoryPoint()
# Supplying positions matching joint count resolves the size validation mismatch:
pt_w.positions = [0.0, 0.0, 0.0, 0.0]
pt_w.velocities = [float(v) for v in self.target_v]
pt_w.time_from_start = Duration(sec=0, nanosec=int(2e8))
traj_w.points = [pt_w]
self.wheel_pub.publish(traj_w)
Patch 2: For JointGroupVelocityController (Recommended)
Switching the publisher interface to simple arrays:

Python
# 1. Add import:
from std_msgs.msg import Float64MultiArray

# 2. In __init__:
self.wheel_pub = self.create_publisher(
    Float64MultiArray, '/wheel_velocity_controller/commands', 10
)

# 3. In timer_callback:
msg_w = Float64MultiArray()
msg_w.data = [float(v) for v in self.target_v]
self.wheel_pub.publish(msg_w)