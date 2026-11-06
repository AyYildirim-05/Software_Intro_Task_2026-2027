#!/usr/bin/env python3

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

INSTRUCTIONS = """
========================================
       Rover Keyboard Teleoperation
========================================
Motion Controls (Hold to move, release to stop):
   w : Drive Forward
   s : Drive Backward
   a : Turn Left (4-Wheel Ackermann)
   d : Turn Right (4-Wheel Ackermann)
   q : Rotate CCW (In-Place Swerve)
   e : Rotate CW (In-Place Swerve)

Speed Tuning:
   ] : Increase wheel velocity
   [ : Decrease wheel velocity

Arm Preset Controls:
   1 : Home Pose
   2 : Reach Pose
   3 : High / Wave Pose

Arm Joint Selection (Chronological Base -> Tip):
   5 : shoulder_yaw
   6 : shoulder_pitch
   7 : elbow_pitch
   8 : elbow_roll
   9 : wrist_pitch
   0 : wrist_roll
   - : Rotate selected joint CCW (-)
   = : Rotate selected joint CW (+)

Exit:
   x / Ctrl+C : Quit
========================================
"""


class RoverTeleopNode(Node):
    def __init__(self):
        super().__init__('rover_teleop')

        # Controllers
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10
        )
        self.steer_pub = self.create_publisher(
            JointTrajectory, '/steering_position_controller/joint_trajectory', 10
        )
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        # Order must match ros2_controllers.yaml joint list: [FL, FR, BL, BR]
        self.wheel_joints = ['fl_wheel', 'fr_wheel', 'bl_wheel', 'br_wheel']
        self.steer_joints = ['fl_swerve_yaw', 'fr_swerve_yaw', 'bl_swerve_yaw', 'br_swerve_yaw']
        self.arm_joints = [
            'shoulder_yaw',
            'shoulder_pitch',
            'elbow_pitch',
            'elbow_roll',
            'wrist_pitch',
            'wrist_roll',
        ]
        self.arm_limits = {
            'shoulder_pitch': (-1.57, 1.57),
            'elbow_pitch': (-1.57, 1.57),
            'wrist_pitch': (-1.57, 1.57),
        }
        self.arm_positions = [0.0] * len(self.arm_joints)
        self.active_arm_joint_idx = 0  # Default to shoulder_yaw (key '5')
        self.arm_step = 0.10           # radians per key press

        # Rover chassis geometry from URDF (meters)
        self.half_wheelbase = 0.445   # L / 2 (front/rear distance from center)
        self.half_track = 0.515       # W / 2 (left/right distance from center)
        self.turn_radius = 1.30       # Centerline turn radius (m) -> ~30 deg inner wheel angle
        self.in_place_angle = math.atan2(self.half_wheelbase, self.half_track)  # ~0.713 rad (40.8 deg)

        self.speed = 5.0              # Base wheel angular velocity (rad/s)
        self.steer_angle = 0.52       # ~30 deg inner steering angle reference
        self.motion_mode = 'stop'

        # Active targets: [FL, FR, BL, BR]
        self.target_v = [0.0, 0.0, 0.0, 0.0]
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self._last_sent_steer = None

        # Timer streaming wheel velocities at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Rover teleop node active.')

    def publish_commands(self, force_steer=False):
        # 1. Wheel Velocity Command
        msg_w = Float64MultiArray()
        msg_w.data = [float(v) for v in self.target_v]
        self.wheel_pub.publish(msg_w)

        # 2. Steering Position Trajectory (only send when changed so we don't reset the controller spline every 50ms)
        if force_steer or self.target_steer != self._last_sent_steer:
            traj_s = JointTrajectory()
            traj_s.joint_names = self.steer_joints
            pt_s = JointTrajectoryPoint()
            pt_s.positions = [float(p) for p in self.target_steer]
            pt_s.time_from_start = Duration(sec=0, nanosec=int(5e7))  # 50 ms response
            traj_s.points = [pt_s]
            self.steer_pub.publish(traj_s)
            self._last_sent_steer = list(self.target_steer)

    def timer_callback(self):
        self.publish_commands(force_steer=False)

    def set_drive_forward(self):
        self.motion_mode = 'forward'
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self.target_v = [self.speed, self.speed, self.speed, self.speed]
        self.publish_commands()

    def set_drive_backward(self):
        self.motion_mode = 'backward'
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self.target_v = [-self.speed, -self.speed, -self.speed, -self.speed]
        self.publish_commands()

    def set_steer_turn(self, direction='left'):
        """
        True 4-Wheel Ackermann steering with differential wheel speeds:
        Inner wheels steer steeper and spin slower; outer wheels steer shallower and spin faster.
        """
        self.motion_mode = f'turn_{direction}'
        r_in_lat = self.turn_radius - self.half_track
        r_out_lat = self.turn_radius + self.half_track

        theta_in = math.atan2(self.half_wheelbase, r_in_lat)
        theta_out = math.atan2(self.half_wheelbase, r_out_lat)

        dist_in = math.hypot(r_in_lat, self.half_wheelbase)
        dist_out = math.hypot(r_out_lat, self.half_wheelbase)

        v_in = self.speed * (dist_in / self.turn_radius)
        v_out = self.speed * (dist_out / self.turn_radius)

        # 'left' ('a') steers toward +Y (FL/BL are inner, FR/BR are outer)
        # 'right' ('d') steers toward -Y (FR/BR are inner, FL/BL are outer)
        if direction == 'left':
            # [FL, FR, BL, BR]
            self.target_steer = [theta_in, theta_out, -theta_in, -theta_out]
            self.target_v = [v_in, v_out, v_in, v_out]
        else:
            # [FL, FR, BL, BR]
            self.target_steer = [-theta_out, -theta_in, theta_out, theta_in]
            self.target_v = [v_out, v_in, v_out, v_in]

        self.publish_commands()

    def set_turn_in_place(self, direction=1):
        """
        Exact chassis aspect-ratio swerve angles for zero-radius turning (tangent to circle).
        FL = -40.8°, FR = +40.8°, BL = +40.8°, BR = -40.8°
        """
        self.motion_mode = 'spin_cw' if direction > 0 else 'spin_ccw'
        a = self.in_place_angle
        self.target_steer = [-a, a, a, -a]
        if direction > 0:  # CW
            self.target_v = [self.speed, -self.speed, self.speed, -self.speed]
        else:              # CCW
            self.target_v = [-self.speed, self.speed, -self.speed, self.speed]
        self.publish_commands()

    def refresh_motion(self):
        if self.motion_mode == 'forward':
            self.set_drive_forward()
        elif self.motion_mode == 'backward':
            self.set_drive_backward()
        elif self.motion_mode == 'turn_left':
            self.set_steer_turn('left')
        elif self.motion_mode == 'turn_right':
            self.set_steer_turn('right')
        elif self.motion_mode == 'spin_cw':
            self.set_turn_in_place(1)
        elif self.motion_mode == 'spin_ccw':
            self.set_turn_in_place(-1)

    def stop(self):
        self.motion_mode = 'stop'
        self.target_v = [0.0, 0.0, 0.0, 0.0]
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self.publish_commands(force_steer=True)

    def send_arm_positions(self, positions, duration_ns=int(5e8)):
        self.arm_positions = [float(p) for p in positions]
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = list(self.arm_positions)
        pt.time_from_start = Duration(sec=0, nanosec=duration_ns)
        traj.points = [pt]
        self.arm_pub.publish(traj)

    def step_active_arm_joint(self, direction):
        idx = self.active_arm_joint_idx
        joint_name = self.arm_joints[idx]
        new_pos = self.arm_positions[idx] + (direction * self.arm_step)
        if joint_name in self.arm_limits:
            low, high = self.arm_limits[joint_name]
            new_pos = max(low, min(high, new_pos))
        self.arm_positions[idx] = new_pos
        self.send_arm_positions(self.arm_positions, duration_ns=int(1e8))
        return joint_name, new_pos


def get_key(settings, timeout=0.02):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        # Drain any queued auto-repeat characters so releasing the key stops immediately
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSANOW, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()

    if not os.isatty(sys.stdin.fileno()):
        node.get_logger().error('Standard input is not a TTY. Run this script in an interactive shell.')
        return

    settings = termios.tcgetattr(sys.stdin)
    print(INSTRUCTIONS)

    last_motion_time = 0.0
    last_motion_key = None
    is_repeating = False

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings, timeout=0.03)
            now = time.monotonic()

            if not key:
                if node.motion_mode != 'stop':
                    # Allow 0.55s for OS initial key-repeat delay, then 0.15s once repeating
                    stop_timeout = 0.15 if is_repeating else 0.55
                    if (now - last_motion_time) > stop_timeout:
                        node.stop()
                        last_motion_key = None
                        is_repeating = False
                        print('\r[IDLE]                              ', end='', flush=True)
                continue

            if key in ('w', 's', 'a', 'd', 'q', 'e'):
                if key == last_motion_key:
                    is_repeating = True
                else:
                    last_motion_key = key
                    is_repeating = False
                last_motion_time = now

            if key == 'w':
                node.set_drive_forward()
                print(f'\r[FORWARD] speed: {node.speed:.1f} rad/s      ', end='', flush=True)

            elif key == 's':
                node.set_drive_backward()
                print(f'\r[BACKWARD] speed: {-node.speed:.1f} rad/s     ', end='', flush=True)

            elif key == 'a':
                node.set_steer_turn('left')
                print(f'\r[TURN LEFT] speed: {node.speed:.1f} rad/s     ', end='', flush=True)

            elif key == 'd':
                node.set_steer_turn('right')
                print(f'\r[TURN RIGHT] speed: {node.speed:.1f} rad/s    ', end='', flush=True)

            elif key == 'q':
                node.set_turn_in_place(direction=-1)
                print('\r[TURN CCW]                          ', end='', flush=True)

            elif key == 'e':
                node.set_turn_in_place(direction=1)
                print('\r[TURN CW]                           ', end='', flush=True)

            elif key == ']':
                node.speed = min(30.0, node.speed + 1.0)
                node.refresh_motion()
                print(f'\r[SPEED UP] current: {node.speed:.1f} rad/s    ', end='', flush=True)

            elif key == '[':
                node.speed = max(0.5, node.speed - 1.0)
                node.refresh_motion()
                print(f'\r[SPEED DOWN] current: {node.speed:.1f} rad/s  ', end='', flush=True)

            elif key in ('5', '6', '7', '8', '9', '0'):
                key_to_idx = {'5': 0, '6': 1, '7': 2, '8': 3, '9': 4, '0': 5}
                idx = key_to_idx[key]
                node.active_arm_joint_idx = idx
                j_name = node.arm_joints[idx]
                j_pos = node.arm_positions[idx]
                print(
                    f'\r[ARM JOINT {idx + 1}/6] {j_name} ({j_pos:+.2f} rad)   ',
                    end='',
                    flush=True,
                )

            elif key in ('=', '+'):
                j_name, j_pos = node.step_active_arm_joint(direction=1)
                print(
                    f'\r[ARM CW +] {j_name}: {j_pos:+.2f} rad          ',
                    end='',
                    flush=True,
                )

            elif key == '-':
                j_name, j_pos = node.step_active_arm_joint(direction=-1)
                print(
                    f'\r[ARM CCW -] {j_name}: {j_pos:+.2f} rad         ',
                    end='',
                    flush=True,
                )

            elif key == '1':
                node.send_arm_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                print('\r[ARM] Home Pose                     ', end='', flush=True)

            elif key == '2':
                node.send_arm_positions([0.0, -0.5, 0.8, 0.0, 0.3, 0.0])
                print('\r[ARM] Reach Pose                    ', end='', flush=True)

            elif key == '3':
                node.send_arm_positions([0.0, -1.0, 1.2, 0.5, 0.2, 1.0])
                print('\r[ARM] High / Wave Pose              ', end='', flush=True)

            elif key in ('x', '\x03'):
                node.stop()
                print('\nExiting rover teleop.')
                break

    except Exception as err:
        print(f'\nRuntime error: {err}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()