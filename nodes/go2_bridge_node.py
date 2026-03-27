#!/usr/bin/env python3
"""MINEBOT-Q Go2 Bridge Node — Dual Confirmation Architecture.

Combines gas alerts (QRB2210), camera detections, and LiDAR proximity
to confirm hazards before commanding the Go2 robot.
"""

import threading
import time
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray

from nodes.go2_sdk_interface import Go2Interface


class Go2BridgeNode(Node):
    def __init__(self):
        super().__init__('go2_bridge_node')

        self.declare_parameter('confirmation_timeout_sec', 2.0)
        self.declare_parameter('retreat_distance_m', 2.0)
        self.declare_parameter('speed_reduction_factor', 0.5)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('lidar_obstacle_distance_m', 1.5)
        self.declare_parameter('normal_linear_speed', 0.4)

        self.timeout = self.get_parameter('confirmation_timeout_sec').value
        self.retreat_dist = self.get_parameter('retreat_distance_m').value
        self.speed_factor = self.get_parameter('speed_reduction_factor').value
        self.sim_mode = self.get_parameter('simulation_mode').value
        self.obstacle_dist = self.get_parameter('lidar_obstacle_distance_m').value
        self.normal_speed = self.get_parameter('normal_linear_speed').value

        self._lock = threading.Lock()

        # Shared state
        self._gas_status = 'SAFE'
        self._gas_timestamp = 0.0
        self._camera_obstacle = False
        self._camera_timestamp = 0.0
        self._lidar_obstacle_near = False
        self._lidar_timestamp = 0.0
        self._robot_state = 'idle'
        self._current_behavior = 'resume'

        # Go2 SDK
        self.go2 = Go2Interface()
        if self.go2.is_mock:
            self.get_logger().warn('Running with Go2InterfaceMock')

        cb_group = ReentrantCallbackGroup()

        # Subscriptions — from QRB2210 via DDS
        self.create_subscription(
            String, '/gas/status', self._gas_status_cb, 10,
            callback_group=cb_group)
        self.create_subscription(
            Float32, '/gas/mq4', self._gas_value_cb, 10,
            callback_group=cb_group)
        self.create_subscription(
            Float32, '/gas/mq7', self._gas_value_cb, 10,
            callback_group=cb_group)
        self.create_subscription(
            Float32, '/gas/mq135', self._gas_value_cb, 10,
            callback_group=cb_group)

        # Subscriptions — local Jetson nodes
        self.create_subscription(
            Detection2DArray, '/camera/detections', self._camera_cb, 10,
            callback_group=cb_group)
        self.create_subscription(
            PointCloud2, '/lidar/points', self._lidar_cb, 10,
            callback_group=cb_group)
        self.create_subscription(
            String, '/go2/state', self._state_cb, 10,
            callback_group=cb_group)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/go2/cmd_vel', 10)
        self.pub_behavior = self.create_publisher(String, '/go2/behavior', 10)
        self.pub_alert = self.create_publisher(String, '/go2/alert', 10)

        # Decision loop at 5 Hz
        self.create_timer(0.2, self._decision_loop, callback_group=cb_group)

        self.get_logger().info(
            f'Go2BridgeNode started — timeout={self.timeout}s, '
            f'retreat={self.retreat_dist}m')

    def _now(self):
        return time.time()

    # ── Callbacks ──────────────────────────────────────────────

    def _gas_status_cb(self, msg: String):
        with self._lock:
            self._gas_status = msg.data.strip().upper()
            self._gas_timestamp = self._now()

    def _gas_value_cb(self, msg: Float32):
        pass  # Values logged for telemetry; decisions use /gas/status

    def _camera_cb(self, msg: Detection2DArray):
        has_obstacle = False
        for det in msg.detections:
            for result in det.results:
                label = result.id
                if label in ('person', 'obstacle', 'crack', 'flood'):
                    has_obstacle = True
                    break
            if has_obstacle:
                break

        with self._lock:
            self._camera_obstacle = has_obstacle
            self._camera_timestamp = self._now()

    def _lidar_cb(self, msg: PointCloud2):
        nearest = self._compute_nearest_distance(msg)
        with self._lock:
            self._lidar_obstacle_near = (nearest < self.obstacle_dist)
            self._lidar_timestamp = self._now()

    def _compute_nearest_distance(self, msg: PointCloud2) -> float:
        point_step = msg.point_step
        data = msg.data
        n_points = msg.width * msg.height
        min_dist = float('inf')

        for i in range(n_points):
            offset = i * point_step
            if offset + 12 > len(data):
                break
            x = struct.unpack_from('f', data, offset)[0]
            y = struct.unpack_from('f', data, offset + 4)[0]
            z = struct.unpack_from('f', data, offset + 8)[0]
            d = math.sqrt(x * x + y * y + z * z)
            if d < min_dist:
                min_dist = d

        return min_dist

    def _state_cb(self, msg: String):
        with self._lock:
            self._robot_state = msg.data.strip().lower()

    # ── Dual Confirmation Decision Loop ───────────────────────

    def _decision_loop(self):
        with self._lock:
            gas = self._gas_status
            gas_ts = self._gas_timestamp
            cam_obs = self._camera_obstacle
            cam_ts = self._camera_timestamp
            lidar_obs = self._lidar_obstacle_near
            lidar_ts = self._lidar_timestamp
            robot_state = self._robot_state

        now = self._now()
        gas_alert = gas in ('WARNING', 'CRITICAL')
        gas_fresh = (now - gas_ts) < self.timeout if gas_ts > 0 else False

        vision_confirmed = False
        if cam_obs and (now - cam_ts) < self.timeout:
            vision_confirmed = True
        if lidar_obs and (now - lidar_ts) < self.timeout:
            vision_confirmed = True

        # Dual confirmation: primary alert + secondary sensor confirmation
        confirmed = gas_fresh and gas_alert and vision_confirmed

        if gas == 'CRITICAL' and confirmed:
            self._execute_behavior('stop', robot_state)
        elif gas == 'WARNING' and confirmed:
            self._execute_behavior('slow', robot_state)
        elif gas == 'SAFE':
            self._execute_behavior('resume', robot_state)

    def _execute_behavior(self, action: str, robot_state: str):
        if action == 'stop' and self._current_behavior != 'stop':
            self._current_behavior = 'stop'
            self.get_logger().warn('DUAL CONFIRMED CRITICAL — STOP + RETREAT')

            self._publish_behavior('stop')
            self._publish_alert('CRITICAL confirmed — executing stop + retreat')
            self.go2.stop()
            self.go2.retreat(self.retreat_dist)
            self._publish_behavior('retreat')

            # Publish zero cmd_vel
            self._publish_cmd_vel(0.0, 0.0)

        elif action == 'slow' and self._current_behavior != 'slow':
            self._current_behavior = 'slow'
            self.get_logger().warn('DUAL CONFIRMED WARNING — reducing speed')

            self._publish_behavior('scan_zone')
            self._publish_alert('WARNING confirmed — speed reduced, scanning')

            reduced = self.normal_speed * self.speed_factor
            self._publish_cmd_vel(reduced, 0.0)
            self.go2.set_velocity(reduced, 0.0)

        elif action == 'resume' and self._current_behavior != 'resume':
            if robot_state in ('stopped', 'retreating'):
                return  # Wait for explicit resume

            self._current_behavior = 'resume'
            self.get_logger().info('SAFE — resuming normal operation')

            self._publish_behavior('resume')
            self.go2.resume()
            self._publish_cmd_vel(self.normal_speed, 0.0)
            self.go2.set_velocity(self.normal_speed, 0.0)

    def _publish_cmd_vel(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub_cmd_vel.publish(twist)

    def _publish_behavior(self, behavior: str):
        msg = String()
        msg.data = behavior
        self.pub_behavior.publish(msg)

    def _publish_alert(self, text: str):
        msg = String()
        msg.data = text
        self.pub_alert.publish(msg)

    # ── Shutdown ──────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('Shutdown — stopping Go2')
        self.go2.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
