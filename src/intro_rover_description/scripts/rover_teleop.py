#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

INSTRUCTIONS = """
========================================
       Rover Keyboard Teleoperation
========================================
Motion Controls:
   w : Drive Forward
   s : Drive Backward
   a : Steer Left
   d : Steer Right
   q : Rotate CCW (in-place)
   e : Rotate CW (in-place)
 space: Stop Wheels & Center Steering

Speed Tuning:
   + / = : Increase speed
   -     : Decrease speed

Arm Preset Controls:
   1 : Home Pose (all zeros)
   2 : Reach Pose
   3 : High / Wave Pose

Exit:
   x / Ctrl+C : Quit
========================================
"""


class RoverTeleopNode(Node):
    def __init__(self):
        super().__init__('rover_teleop')

        self.wheel_pub = self.create_publisher(
            JointTrajectory, '/wheel_velocity_controller/joint_trajectory', 10
        )
        self.steer_pub = self.create_publisher(
            JointTrajectory, '/steering_position_controller/joint_trajectory', 10
        )
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

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

        self.speed = 5.0  # rad/s
        self.steer_angle = 0.52  # ~30 deg

        # Active state targets
        self.target_v = [0.0, 0.0, 0.0, 0.0]
        self.target_steer = [0.0, 0.0, 0.0, 0.0]

        # Continuous publisher timer (20 Hz) so controllers never starve
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Rover teleop continuous streaming node initialized.')

    def timer_callback(self):
        # Stream wheel velocity
        traj_w = JointTrajectory()
        traj_w.joint_names = self.wheel_joints
        pt_w = JointTrajectoryPoint()
        pt_w.velocities = [float(v) for v in self.target_v]
        pt_w.time_from_start = Duration(sec=0, nanosec=int(1e8))  # 100ms horizon
        traj_w.points.append(pt_w)
        self.wheel_pub.publish(traj_w)

        # Stream steer positions
        traj_s = JointTrajectory()
        traj_s.joint_names = self.steer_joints
        pt_s = JointTrajectoryPoint()
        pt_s.positions = [float(p) for p in self.target_steer]
        pt_s.time_from_start = Duration(sec=0, nanosec=int(1e8))
        traj_s.points.append(pt_s)
        self.steer_pub.publish(traj_s)

    def set_drive(self, v_fl, v_fr, v_bl, v_br, steer_all=0.0):
        self.target_v = [v_fl, v_fr, v_bl, v_br]
        self.target_steer = [steer_all, steer_all, steer_all, steer_all]

    def set_turn_in_place(self, direction=1):
        # Swerve 45 deg angles for in-place yaw
        self.target_steer = [0.785, -0.785, -0.785, 0.785]
        if direction > 0:  # CW
            self.target_v = [self.speed, -self.speed, self.speed, -self.speed]
        else:  # CCW
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
        traj.points.append(pt)
        self.arm_pub.publish(traj)


def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()

    if not os.isatty(sys.stdin.fileno()):
        node.get_logger().error('Terminal is not interactive. Run rover_teleop in a terminal.')
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
                node.set_drive(node.speed, node.speed, node.speed, node.speed, steer_all=0.0)
                print(f'\r[FORWARD] speed: {node.speed:.1f} rad/s', end='', flush=True)

            elif key == 's':
                node.set_drive(-node.speed, -node.speed, -node.speed, -node.speed, steer_all=0.0)
                print(f'\r[BACKWARD] speed: {-node.speed:.1f} rad/s', end='', flush=True)

            elif key == 'a':
                node.set_drive(node.speed, node.speed, node.speed, node.speed, steer_all=node.steer_angle)
                print(f'\r[STEER LEFT] angle: {node.steer_angle:.2f} rad', end='', flush=True)

            elif key == 'd':
                node.set_drive(node.speed, node.speed, node.speed, node.speed, steer_all=-node.steer_angle)
                print(f'\r[STEER RIGHT] angle: {-node.steer_angle:.2f} rad', end='', flush=True)

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
                node.speed = min(20.0, node.speed + 1.0)
                print(f'\r[SPEED UP] current speed: {node.speed:.1f} rad/s', end='', flush=True)

            elif key == '-':
                node.speed = max(1.0, node.speed - 1.0)
                print(f'\r[SPEED DOWN] current speed: {node.speed:.1f} rad/s', end='', flush=True)

            elif key == '1':
                node.send_arm_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                print('\r[ARM] Home Pose', end='', flush=True)

            elif key == '2':
                node.send_arm_positions([0.0, -0.5, 0.8, 0.0, 0.3, 0.0])
                print('\r[ARM] Reach Pose', end='', flush=True)

            elif key == '3':
                node.send_arm_positions([0.0, -1.0, 1.2, 0.5, 0.2, 1.0])
                print('\r[ARM] High / Wave Pose', end='', flush=True)

            elif key == 'x' or key == '\x03':
                node.stop()
                print('\nExiting rover teleop.')
                break

    except Exception as e:
        print(f'\nError: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()