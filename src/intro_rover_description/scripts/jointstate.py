#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RoverDanceJointStatePublisher(Node):
    def __init__(self):
        super().__init__('rover_dance_joint_state_publisher')

        self.joint_names = [
            'shoulder_yaw',
            'shoulder_pitch',
            'elbow_pitch',
            'elbow_roll',
            'wrist_pitch',
            'wrist_roll',
            'fl_swerve_yaw',
            'fr_swerve_yaw',
            'bl_swerve_yaw',
            'br_swerve_yaw',
            'fl_wheel',
            'fr_wheel',
            'bl_wheel',
            'br_wheel'
        ]

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.time = 0.0

        self.get_logger().info('Rover dance joint state publisher initialized and running!')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Use a continuous phase variable so motion never wraps discontinuously.
        t = self.time
        arm_base = 1.8 * math.sin(0.9 * t)
        shoulder_yaw = 0.55 * math.sin(0.9 * t)
        shoulder_pitch = -0.35 + 0.25 * math.cos(1.2 * t)
        elbow_pitch = 0.65 * math.sin(1.5 * t + 0.8)
        elbow_roll = 0.75 * math.sin(1.1 * t + 1.3)
        wrist_pitch = 0.42 * math.cos(1.8 * t + 0.4)
        wrist_roll = 0.90 * math.sin(2.2 * t + 1.0)

        fl_yaw = 0.45 * math.sin(1.1 * t)
        fr_yaw = -0.45 * math.sin(1.1 * t + 0.7)
        bl_yaw = -0.45 * math.sin(1.1 * t + 1.3)
        br_yaw = 0.45 * math.sin(1.1 * t + 2.0)

        # Keep wheel angles moving continuously with no modulo wrap.
        wheel_spin = 2.0 * t

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
            wheel_spin,
            wheel_spin + 0.4,
            wheel_spin + 1.1,
            wheel_spin + 1.7,
        ]
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)

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