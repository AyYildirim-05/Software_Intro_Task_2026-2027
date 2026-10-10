#!/usr/bin/env python3

import argparse
import math
import os
import subprocess
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class GazeboKeyboardMover(Node):
    def __init__(self, world_name: str, entity_name: str, step: float, turn_step: float):
        super().__init__('gazebo_keyboard_mover')
        self.world_name = world_name
        self.entity_name = entity_name
        self.step = step
        self.turn_step = turn_step

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.pose_service = None

        for srv_name in [
            f'/world/{self.world_name}/set_pose',
            f'/world/{self.world_name}/set_entity_pose',
        ]:
            client = self.create_client(SetEntityPose, srv_name)
            if client.wait_for_service(timeout_sec=1.0):
                self.pose_service = client
                self.get_logger().info(f'Using Gazebo pose service: {srv_name}')
                break

        if self.pose_service is None:
            self.get_logger().warning(
                'No ROS Gazebo pose service is available. Falling back to ros_gz_sim set_entity_pose CLI.'
            )

        self.get_logger().info(
            'Keyboard controls active: w=forward, s=backward, a=left, d=right, q=turn left, e=turn right, x=exit'
        )
        self.get_logger().info(
            f'World: {self.world_name} | Entity: {self.entity_name} | step: {self.step:.2f} m | turn step: {self.turn_step:.2f} rad'
        )

    def _yaw_to_quaternion(self, yaw: float):
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def _send_pose_via_cli(self):
        cmd = [
            'ros2',
            'run',
            'ros_gz_sim',
            'set_entity_pose',
            '--name',
            self.entity_name,
            '--pos',
            str(self.x),
            str(self.y),
            '0.5',
            '--euler',
            '0.0',
            '0.0',
            str(self.yaw),
        ]

        completed = subprocess.run(cmd, capture_output=True, text=True)
        if completed.returncode != 0:
            self.get_logger().error(
                'ros_gz_sim set_entity_pose failed: %s',
                completed.stderr.strip() or completed.stdout.strip() # type: ignore
            )
            return False
        return True

    def _send_pose(self):
        if self.pose_service is not None:
            req = SetEntityPose.Request()
            req.entity = Entity()
            req.entity.name = self.entity_name
            req.entity.type = Entity.MODEL

            pose = Pose()
            pose.position.x = self.x
            pose.position.y = self.y
            pose.position.z = 0.5

            qx, qy, qz, qw = self._yaw_to_quaternion(self.yaw)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            req.pose = pose

            future = self.pose_service.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            result = future.result()
            if result is None:
                self.get_logger().error(f'Failed to update pose for {self.entity_name}: {future.exception()}')
                return False
            if not result.success:
                self.get_logger().error(f'Gazebo rejected pose update for {self.entity_name}.')
                return False

            return True

        return self._send_pose_via_cli()

    def move(self, dx: float, dy: float, d_yaw: float):
        self.x += dx
        self.y += dy
        self.yaw += d_yaw
        return self._send_pose()

    def run(self):
        if not os.isatty(sys.stdin.fileno()):
            self.get_logger().warning(
                'No TTY detected; keyboard control is unavailable in this context. '
                'Run this node in an interactive terminal to move the rover with w/s/a/d/q/e/x.'
            )
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if not ch:
                    continue

                key = ch.lower()

                if key == 'w':
                    self.move(self.step, 0.0, 0.0)
                elif key == 's':
                    self.move(-self.step, 0.0, 0.0)
                elif key == 'a':
                    self.move(0.0, self.step, 0.0)
                elif key == 'd':
                    self.move(0.0, -self.step, 0.0)
                elif key == 'q':
                    self.move(0.0, 0.0, self.turn_step)
                elif key == 'e':
                    self.move(0.0, 0.0, -self.turn_step)
                elif key == 'x':
                    self.get_logger().info('Exiting keyboard control.')
                    break
                else:
                    # ignore unsupported keys, but keep the terminal responsive
                    continue
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    argv = remove_ros_args(sys.argv[1:]) if args is None else remove_ros_args(args)

    parser = argparse.ArgumentParser(description='Keyboard control for robot motion in Gazebo.')
    parser.add_argument('--world', default='empty', help='Gazebo world name (default: empty)')
    parser.add_argument('--entity', default='intro_rover', help='Gazebo model name to move (default: intro_rover)')
    parser.add_argument('--step', type=float, default=0.25, help='Linear movement step in meters')
    parser.add_argument('--turn-step', type=float, default=0.26, help='Angular turn step in radians')
    parsed, _ = parser.parse_known_args(argv)

    rclpy.init(args=argv)
    node = GazeboKeyboardMover(parsed.world, parsed.entity, parsed.step, parsed.turn_step)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
