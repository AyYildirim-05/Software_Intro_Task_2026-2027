#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RoverDanceJointStatePublisher(Node):
    def __init__(self):
        super().__init__('rover_dance_joint_state_publisher')

        # Exact movable joints from your rover's URDF tree
        self.joint_names = [
            # Arm Joints (6-DOF) -- names match URDF
            'shoulder_yaw',
            'shoulder_pitch',
            'elbow_pitch',
            'elbow_roll',
            'wrist_pitch',
            'wrist_roll',
            # Swerve Steering Yaw Joints
            'fl_swerve_yaw',
            'fr_swerve_yaw',
            'bl_swerve_yaw',
            'br_swerve_yaw',
            # Wheel Continuous Joints
            'fl_wheel',
            'fr_wheel',
            'bl_wheel',
            'br_wheel'
        ]

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # 50 Hz loop timer (dt = 0.02s)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.time = 0.0

        self.get_logger().info('Rover dance joint state publisher initialized and running!')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Dance rhythm timing (120 BPM base tempo)
        bpm = 120.0
        freq = (bpm / 60.0) * 2.0 * math.pi
        t = self.time

        # Choreography calculation
        shoulder_yaw = 0.7 * math.sin(freq * 0.5 * t)
        shoulder_pitch = -0.4 + 0.3 * math.cos(freq * t)
        elbow_pitch = 0.6 * math.sin(freq * t)
        elbow_roll = 0.8 * math.sin(freq * 0.5 * t + math.pi / 2.0)
        wrist_pitch = 0.4 * math.cos(freq * 2.0 * t)
        wrist_roll = 1.2 * math.sin(freq * 2.0 * t)

        # Swerve wiggles
        fl_yaw = 0.4 * math.sin(freq * t)
        fr_yaw = -0.4 * math.sin(freq * t)
        bl_yaw = -0.4 * math.sin(freq * t)
        br_yaw = 0.4 * math.sin(freq * t)

        # Wheel spins
        wheel_pos = (freq * 0.5 * t) % (2.0 * math.pi)

        msg.position = [
            shoulder_yaw,
            shoulder_pitch,
            elbow_pitch,
            elbow_roll,
            wrist_pitch,
            wrist_roll,
            fl_yaw,
            fr_yaw,
            bl_yaw,
            br_yaw,
            wheel_pos,
            wheel_pos,
            wheel_pos,
            wheel_pos
        ]

        self.publisher_.publish(msg)
        self.time += self.timer_period


def main(args=None):
    rclpy.init(args=args)
    node = RoverDanceJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()