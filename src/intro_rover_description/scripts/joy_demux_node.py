#!/usr/bin/env python3

import math
import subprocess

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JoyDemuxNode(Node):
    """Map PS4 /joy inputs to /cmd_vel and ros2_control drivetrain and arm commands."""

    def __init__(self):
        super().__init__('joy_demux_node')

        # Declare parameters (loaded from config/ps4_controller.yaml)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('drive_forward_axis', 1)
        self.declare_parameter('drive_turn_axis', 0)
        self.declare_parameter('max_wheel_speed', 8.0)
        self.declare_parameter('arm_stick_horizontal_axis', 3)
        self.declare_parameter('arm_stick_vertical_axis', 4)
        self.declare_parameter('arm_joint_speed', 1.5)
        self.declare_parameter('arm_joint_buttons', [0, 1, 2, 3, 4, 5])
        self.declare_parameter('arm_reset_button', 9)
        self.declare_parameter('spin_ccw_button', 6)
        self.declare_parameter('spin_cw_button', 7)

        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.drive_fwd_axis = (
            self.get_parameter('drive_forward_axis').get_parameter_value().integer_value
        )
        self.drive_turn_axis = (
            self.get_parameter('drive_turn_axis').get_parameter_value().integer_value
        )
        self.max_wheel_speed = (
            self.get_parameter('max_wheel_speed').get_parameter_value().double_value
        )
        self.arm_h_axis = (
            self.get_parameter('arm_stick_horizontal_axis').get_parameter_value().integer_value
        )
        self.arm_v_axis = (
            self.get_parameter('arm_stick_vertical_axis').get_parameter_value().integer_value
        )
        self.arm_joint_speed = (
            self.get_parameter('arm_joint_speed').get_parameter_value().double_value
        )
        self.arm_buttons = list(
            self.get_parameter('arm_joint_buttons').get_parameter_value().integer_array_value
        )
        self.arm_reset_btn = (
            self.get_parameter('arm_reset_button').get_parameter_value().integer_value
        )
        self.spin_ccw_btn = (
            self.get_parameter('spin_ccw_button').get_parameter_value().integer_value
        )
        self.spin_cw_btn = (
            self.get_parameter('spin_cw_button').get_parameter_value().integer_value
        )

        # Subscribe to /joy from joy_node
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publish /cmd_vel (matches architecture diagram)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publish to ros2_control controllers
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10
        )
        self.steer_pub = self.create_publisher(
            JointTrajectory, '/steering_position_controller/joint_trajectory', 10
        )
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        # Joint names matching rover_controllers.yaml
        self.steer_joints = ['fl_swerve_yaw', 'fr_swerve_yaw', 'bl_swerve_yaw', 'br_swerve_yaw']

        # Chronological arm joints (Base -> End-Effector)
        self.arm_joints = [
            'shoulder_yaw',    # 1. Base yaw       (continuous)
            'shoulder_pitch',  # 2. Shoulder pitch (revolute: -1.57 to 1.57)
            'elbow_pitch',     # 3. Elbow pitch    (revolute: -1.57 to 1.57)
            'elbow_roll',      # 4. Elbow roll     (continuous)
            'wrist_pitch',     # 5. Wrist pitch    (revolute: -1.57 to 1.57)
            'wrist_roll',      # 6. Wrist roll     (continuous)
        ]
        self.arm_limits = {
            'shoulder_pitch': (-1.57, 1.57),
            'elbow_pitch': (-1.57, 1.57),
            'wrist_pitch': (-1.57, 1.57),
        }
        self.button_labels = ['Cross (X)', 'Circle (O)', 'Triangle', 'Square', 'L1', 'R1']

        # Rover chassis geometry (meters)
        self.half_wheelbase = 0.445
        self.half_track = 0.515
        self.turn_radius = 1.30
        self.in_place_angle = math.atan2(self.half_wheelbase, self.half_track)

        # State tracking
        self.active_arm_joint_idx = 0  # Default to first joint (shoulder_yaw)
        self._prev_buttons = []
        self.arm_positions = [0.0] * len(self.arm_joints)
        self.right_stick_val = 0.0

        self.target_v = [0.0, 0.0, 0.0, 0.0]
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self._last_sent_steer = None

        # 20 Hz control loop (dt = 0.05s)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            'PS4 Joy Demux Node started.\n'
            '  - Left Stick: Drive (Up/Down = Forward/Back, Left/Right = Turn/Spin)\n'
            '  - Arm Joint Buttons (Chronological Base -> Tip):\n'
            '      [0] Cross (X)  -> shoulder_yaw\n'
            '      [1] Circle (O) -> shoulder_pitch\n'
            '      [2] Triangle   -> elbow_pitch\n'
            '      [3] Square     -> elbow_roll\n'
            '      [4] L1         -> wrist_pitch\n'
            '      [5] R1         -> wrist_roll\n'
            '  - Right Stick: Rotate selected arm joint (+ = Clockwise, - = Counter-Clockwise)'
        )

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return float(value)

    def joy_callback(self, msg):
        # 1. Check arm joint selection buttons in chronological order
        for joint_idx, btn_idx in enumerate(self.arm_buttons):
            if btn_idx < len(msg.buttons):
                was_pressed = (
                    btn_idx < len(self._prev_buttons) and self._prev_buttons[btn_idx] == 1
                )
                is_pressed = msg.buttons[btn_idx] == 1
                if is_pressed and not was_pressed and self.active_arm_joint_idx != joint_idx:
                    self.active_arm_joint_idx = joint_idx
                    label = (
                        self.button_labels[joint_idx]
                        if joint_idx < len(self.button_labels)
                        else f'Button {btn_idx}'
                    )
                    self.get_logger().info(
                        f'Selected Arm Joint [{joint_idx + 1}/6]: '
                        f'{self.arm_joints[joint_idx]} ({label})'
                    )
                elif is_pressed:
                    self.active_arm_joint_idx = joint_idx

        # Optional reset button (Options)
        if self.arm_reset_btn < len(msg.buttons):
            was_reset = (
                self.arm_reset_btn < len(self._prev_buttons)
                and self._prev_buttons[self.arm_reset_btn] == 1
            )
            if msg.buttons[self.arm_reset_btn] == 1 and not was_reset:
                self.reset_rover()
                self.get_logger().info('Reset rover pose and all joints to home position.')

        self._prev_buttons = list(msg.buttons)

        # 2. Read Right Stick for arm joint rotation (+ = clockwise, - = counter-clockwise)
        r_h = (
            self._apply_deadzone(msg.axes[self.arm_h_axis])
            if self.arm_h_axis < len(msg.axes)
            else 0.0
        )
        r_v = (
            self._apply_deadzone(msg.axes[self.arm_v_axis])
            if self.arm_v_axis < len(msg.axes)
            else 0.0
        )
        # Pushing Right (-r_h) or Up (+r_v) yields positive (+) clockwise movement
        stick_x_cw = -r_h
        stick_y_cw = r_v
        self.right_stick_val = stick_x_cw if abs(stick_x_cw) >= abs(stick_y_cw) else stick_y_cw

        # 3. Read L2/R2 spin buttons and Left Stick for drivetrain movement
        spin_ccw = (
            self.spin_ccw_btn < len(msg.buttons) and msg.buttons[self.spin_ccw_btn] == 1
        )
        spin_cw = (
            self.spin_cw_btn < len(msg.buttons) and msg.buttons[self.spin_cw_btn] == 1
        )

        fwd = (
            self._apply_deadzone(msg.axes[self.drive_fwd_axis])
            if self.drive_fwd_axis < len(msg.axes)
            else 0.0
        )
        turn = (
            self._apply_deadzone(msg.axes[self.drive_turn_axis])
            if self.drive_turn_axis < len(msg.axes)
            else 0.0
        )

        # Publish /cmd_vel Twist message
        twist = Twist()
        twist.linear.x = float(fwd * self.max_wheel_speed * 0.14)  # m/s (wheel radius = 0.14m)
        if spin_ccw and not spin_cw:
            twist.angular.z = 1.5
        elif spin_cw and not spin_ccw:
            twist.angular.z = -1.5
        else:
            twist.angular.z = float(turn * 1.5)                    # rad/s
        self.cmd_vel_pub.publish(twist)

        # Compute wheel velocities & steering angles from L2/R2 or Left Stick
        self.compute_drivetrain_targets(fwd, turn, spin_ccw, spin_cw)

    def compute_drivetrain_targets(self, fwd, turn, spin_ccw=False, spin_cw=False):
        # Dedicated L2 (CCW / 'q') and R2 (CW / 'e') in-place rotation
        if spin_ccw and not spin_cw:
            a = self.in_place_angle
            v = self.max_wheel_speed
            self.target_steer = [-a, a, a, -a]
            self.target_v = [-v, v, -v, v]
            return
        if spin_cw and not spin_ccw:
            a = self.in_place_angle
            v = self.max_wheel_speed
            self.target_steer = [-a, a, a, -a]
            self.target_v = [v, -v, v, -v]
            return

        if fwd == 0.0 and turn == 0.0:
            self.target_v = [0.0, 0.0, 0.0, 0.0]
            self.target_steer = [0.0, 0.0, 0.0, 0.0]
            return

        mag = min(math.hypot(fwd, turn), 1.0)
        speed = mag * self.max_wheel_speed
        base_speed = speed if fwd >= 0.0 else -speed

        # 1. Pure vertical stick deflection -> Straight drive
        if abs(turn) < 0.20:
            self.target_steer = [0.0, 0.0, 0.0, 0.0]
            self.target_v = [base_speed, base_speed, base_speed, base_speed]
            return

        # 2. Left/Right or Diagonal stick deflection -> Fixed-geometry 4-Wheel Ackermann turn
        r_in_lat = self.turn_radius - self.half_track
        r_out_lat = self.turn_radius + self.half_track

        theta_in = math.atan2(self.half_wheelbase, r_in_lat)
        theta_out = math.atan2(self.half_wheelbase, r_out_lat)

        dist_in = math.hypot(r_in_lat, self.half_wheelbase)
        dist_out = math.hypot(r_out_lat, self.half_wheelbase)

        v_in = base_speed * (dist_in / self.turn_radius)
        v_out = base_speed * (dist_out / self.turn_radius)

        if turn > 0.0:  # Left (+Y: FL/BL inner, FR/BR outer)
            self.target_steer = [theta_in, theta_out, -theta_in, -theta_out]
            self.target_v = [v_in, v_out, v_in, v_out]
        else:           # Right (-Y: FR/BR inner, FL/BL outer)
            self.target_steer = [-theta_out, -theta_in, theta_out, theta_in]
            self.target_v = [v_out, v_in, v_out, v_in]

    def publish_arm_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in self.arm_positions]
        pt.time_from_start = Duration(sec=0, nanosec=int(5e7))  # 50 ms
        traj.points = [pt]
        self.arm_pub.publish(traj)

    def reset_rover(self):
        self.target_v = [0.0, 0.0, 0.0, 0.0]
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self._last_sent_steer = None
        self.arm_positions = [0.0] * len(self.arm_joints)
        self.publish_arm_trajectory()
        subprocess.Popen(
            [
                'gz', 'service',
                '-s', '/world/mars/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req',
                'name: "intro_rover", position: {x: 0.0, y: 0.0, z: 0.5}, '
                'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def control_loop(self):
        # 1. Publish wheel velocities
        msg_w = Float64MultiArray()
        msg_w.data = [float(v) for v in self.target_v]
        self.wheel_pub.publish(msg_w)

        # 2. Publish steering angles when changed
        if self.target_steer != self._last_sent_steer:
            traj_s = JointTrajectory()
            traj_s.joint_names = self.steer_joints
            pt_s = JointTrajectoryPoint()
            pt_s.positions = [float(p) for p in self.target_steer]
            pt_s.time_from_start = Duration(sec=0, nanosec=int(5e7))
            traj_s.points = [pt_s]
            self.steer_pub.publish(traj_s)
            self._last_sent_steer = list(self.target_steer)

        # 3. Update active arm joint if Right Stick is deflected
        if self.right_stick_val != 0.0:
            idx = self.active_arm_joint_idx
            joint_name = self.arm_joints[idx]
            # Positive right_stick_val -> clockwise (+), Negative -> counter-clockwise (-)
            delta = self.right_stick_val * self.arm_joint_speed * self.dt
            new_pos = self.arm_positions[idx] + delta

            # Clamp to URDF joint limits if revolute
            if joint_name in self.arm_limits:
                low, high = self.arm_limits[joint_name]
                new_pos = max(low, min(high, new_pos))

            self.arm_positions[idx] = new_pos
            self.publish_arm_trajectory()


def main(args=None):
    rclpy.init(args=args)
    node = JoyDemuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
