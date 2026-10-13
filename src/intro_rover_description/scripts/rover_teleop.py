#!/usr/bin/env python3

import os
import select
import sys
import termios
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
Motion Controls:
   w : Drive Forward
   s : Drive Backward
   a : Turn Left (Front angled, Rear opposed)
   d : Turn Right (Front angled, Rear opposed)
   q : Rotate CCW (In-Place Swerve)
   e : Rotate CW (In-Place Swerve)
 space: Stop Wheels & Straighten Steering

Speed Tuning:
   + / = : Increase wheel velocity
   -     : Decrease wheel velocity

Arm Preset Controls:
   1 : Home Pose
   2 : Reach Pose
   3 : High / Wave Pose

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

        # Order must match ros2_controllers.yaml joint list
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

        self.speed = 5.0        # rad/s
        self.steer_angle = 0.52 # ~30 deg

        # Active targets: [FL, FR, BL, BR]
        self.target_v = [0.0, 0.0, 0.0, 0.0]
        self.target_steer = [0.0, 0.0, 0.0, 0.0]

        # Timer streaming at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Rover teleop node active.')

    def timer_callback(self):
        # 1. Wheel Velocity Command
        msg_w = Float64MultiArray()
        msg_w.data = [float(v) for v in self.target_v]
        self.wheel_pub.publish(msg_w)

        # 2. Steering Position Trajectory
        traj_s = JointTrajectory()
        traj_s.joint_names = self.steer_joints
        pt_s = JointTrajectoryPoint()
        pt_s.positions = [float(p) for p in self.target_steer]
        pt_s.time_from_start = Duration(sec=0, nanosec=int(2e8))
        traj_s.points = [pt_s]
        self.steer_pub.publish(traj_s)

    def set_drive_forward(self):
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self.target_v = [self.speed, self.speed, self.speed, self.speed]

    def set_drive_backward(self):
        self.target_steer = [0.0, 0.0, 0.0, 0.0]
        self.target_v = [-self.speed, -self.speed, -self.speed, -self.speed]

    def set_steer_turn(self, direction='left'):
        """
        Coordinates 4-wheel steering:
        Front wheels turn into the angle, rear wheels oppose for a true turn arc.
        """
        angle = self.steer_angle if direction == 'left' else -self.steer_angle
        self.target_steer = [angle, angle, -angle, -angle]
        self.target_v = [self.speed, self.speed, self.speed, self.speed]

    def set_turn_in_place(self, direction=1):
        """
        Swerve module angles for zero-radius turning (tangent to circle).
        FL = -45°, FR = 45°, BL = 45°, BR = -45°
        """
        self.target_steer = [-0.785, 0.785, 0.785, -0.785]
        if direction > 0:  # CW
            self.target_v = [self.speed, -self.speed, self.speed, -self.speed]
        else:              # CCW
            self.target_v = [-self.speed, self.speed, -self.speed, self.speed]

    def stop(self):
        self.target_v = [0.0, 0.0, 0.0, 0.0]
        self.target_steer = [0.0, 0.0, 0.0, 0.0]

    def send_arm_positions(self, positions):
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        traj.points = [pt]
        self.arm_pub.publish(traj)


def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()

    if not os.isatty(sys.stdin.fileno()):
        node.get_logger().error('Standard input is not a TTY. Run this script in an interactive shell.')
        return

    settings = termios.tcgetattr(sys.stdin)
    print(INSTRUCTIONS)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings, timeout=0.05)

            if not key:
                continue

            if key == 'w':
                node.set_drive_forward()
                print(f'\r[FORWARD] speed: {node.speed:.1f} rad/s', end='', flush=True)

            elif key == 's':
                node.set_drive_backward()
                print(f'\r[BACKWARD] speed: {-node.speed:.1f} rad/s', end='', flush=True)

            elif key == 'a':
                node.set_steer_turn('left')
                print(f'\r[TURN LEFT] angle: {node.steer_angle:.2f} rad', end='', flush=True)

            elif key == 'd':
                node.set_steer_turn('right')
                print(f'\r[TURN RIGHT] angle: {-node.steer_angle:.2f} rad', end='', flush=True)

            elif key == 'q':
                node.set_turn_in_place(direction=-1)
                print('\r[TURN CCW]', end='', flush=True)

            elif key == 'e':
                node.set_turn_in_place(direction=1)
                print('\r[TURN CW]', end='', flush=True)

            elif key == ' ':
                node.stop()
                print('\r[STOPPED]', end='', flush=True)

            elif key in ('+', '='):
                node.speed = min(30.0, node.speed + 1.0)
                print(f'\r[SPEED UP] current: {node.speed:.1f} rad/s', end='', flush=True)

            elif key == '-':
                node.speed = max(0.5, node.speed - 1.0)
                print(f'\r[SPEED DOWN] current: {node.speed:.1f} rad/s', end='', flush=True)

            elif key == '1':
                node.send_arm_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                print('\r[ARM] Home Pose', end='', flush=True)

            elif key == '2':
                node.send_arm_positions([0.0, -0.5, 0.8, 0.0, 0.3, 0.0])
                print('\r[ARM] Reach Pose', end='', flush=True)

            elif key == '3':
                node.send_arm_positions([0.0, -1.0, 1.2, 0.5, 0.2, 1.0])
                print('\r[ARM] High / Wave Pose', end='', flush=True)

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