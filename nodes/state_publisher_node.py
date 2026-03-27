#!/usr/bin/env python3
"""MINEBOT-Q State Publisher Node — publishes Go2 state, pose, and battery."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped

from nodes.go2_sdk_interface import Go2Interface


class StatePublisherNode(Node):
    def __init__(self):
        super().__init__('state_publisher_node')

        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('battery_warn_level', 20.0)

        rate = self.get_parameter('publish_rate_hz').value
        self.battery_warn = self.get_parameter('battery_warn_level').value

        self.go2 = Go2Interface()

        self.pub_state = self.create_publisher(String, '/go2/state', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/go2/pose', 10)
        self.pub_battery = self.create_publisher(Float32, '/go2/battery', 10)

        self.timer = self.create_timer(1.0 / rate, self._publish_state)
        self._battery_warned = False

        self.get_logger().info(
            f'StatePublisherNode started at {rate} Hz')

    def _publish_state(self):
        now = self.get_clock().now().to_msg()

        # State
        state_msg = String()
        state_msg.data = self.go2.get_state()
        self.pub_state.publish(state_msg)

        # Pose
        pose_data = self.go2.get_pose()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = pose_data['x']
        pose_msg.pose.position.y = pose_data['y']
        pose_msg.pose.position.z = pose_data['z']

        yaw = pose_data['yaw']
        pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_pose.publish(pose_msg)

        # Battery
        battery = self.go2.get_battery()
        bat_msg = Float32()
        bat_msg.data = battery
        self.pub_battery.publish(bat_msg)

        if battery >= 0 and battery < self.battery_warn and not self._battery_warned:
            self.get_logger().warn(
                f'LOW BATTERY: {battery:.1f}% (threshold: {self.battery_warn}%)')
            self._battery_warned = True
        elif battery >= self.battery_warn:
            self._battery_warned = False


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
