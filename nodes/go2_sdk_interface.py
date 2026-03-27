#!/usr/bin/env python3
"""MINEBOT-Q Go2 SDK Interface — abstraction over unitree_sdk2_python."""

import time
import math
import logging

logger = logging.getLogger('go2_sdk_interface')


class Go2Interface:
    """Abstraction for Unitree Go2 robot control via internal Jetson Orin NX."""

    MAX_LINEAR_VEL = 0.5   # m/s
    MAX_ANGULAR_VEL = 0.8  # rad/s

    def __init__(self):
        self._state = 'idle'
        try:
            from unitree_sdk2_python.core.channel import ChannelFactoryInitialize
            from unitree_sdk2_python.go2.sport.sport_client import SportClient

            ChannelFactoryInitialize(0)
            self._client = SportClient()
            self._client.Init()
            self._mock = False
            logger.info('Go2Interface: unitree_sdk2_python initialized')
        except ImportError:
            logger.warning(
                'unitree_sdk2_python not available — using Go2InterfaceMock')
            self._client = None
            self._mock = True

    @property
    def is_mock(self):
        return self._mock

    def stop(self) -> bool:
        self._state = 'stopped'
        if self._mock:
            logger.info('[MOCK] stop()')
            return True
        try:
            self._client.StopMove()
            return True
        except Exception as e:
            logger.error(f'stop() failed: {e}')
            return False

    def retreat(self, distance_m: float = 2.0) -> bool:
        self._state = 'retreating'
        speed = 0.3  # m/s conservative emergency speed
        duration = distance_m / speed

        if self._mock:
            logger.info(
                f'[MOCK] retreat({distance_m}m) at {speed}m/s '
                f'for {duration:.1f}s')
            self._state = 'stopped'
            return True

        try:
            start = time.time()
            while time.time() - start < duration:
                self._client.Move(-speed, 0.0, 0.0)
                time.sleep(0.05)
            self._client.StopMove()
            self._state = 'stopped'
            return True
        except Exception as e:
            logger.error(f'retreat() failed: {e}')
            self._client.StopMove()
            self._state = 'stopped'
            return False

    def set_velocity(self, linear_x: float, angular_z: float) -> bool:
        linear_x = max(-self.MAX_LINEAR_VEL,
                       min(self.MAX_LINEAR_VEL, linear_x))
        angular_z = max(-self.MAX_ANGULAR_VEL,
                        min(self.MAX_ANGULAR_VEL, angular_z))

        if self._state in ('stopped', 'retreating'):
            logger.warning(
                f'set_velocity blocked — current state: {self._state}')
            return False

        self._state = 'moving'
        if self._mock:
            logger.info(
                f'[MOCK] set_velocity(linear={linear_x:.2f}, '
                f'angular={angular_z:.2f})')
            return True

        try:
            self._client.Move(linear_x, 0.0, angular_z)
            return True
        except Exception as e:
            logger.error(f'set_velocity() failed: {e}')
            return False

    def scan_zone(self) -> dict:
        results = {}
        step_deg = 45
        n_steps = 360 // step_deg

        if self._mock:
            logger.info('[MOCK] scan_zone() — 360° in 45° steps')
            for i in range(n_steps):
                results[i * step_deg] = {'clear': True}
            self._state = 'idle'
            return results

        try:
            for i in range(n_steps):
                angle_rad = math.radians(step_deg)
                turn_time = angle_rad / self.MAX_ANGULAR_VEL
                start = time.time()
                while time.time() - start < turn_time:
                    self._client.Move(0.0, 0.0, self.MAX_ANGULAR_VEL)
                    time.sleep(0.05)
                self._client.StopMove()
                time.sleep(0.3)
                results[i * step_deg] = {'clear': True}

            self._state = 'idle'
            return results
        except Exception as e:
            logger.error(f'scan_zone() failed: {e}')
            self._state = 'idle'
            return results

    def get_pose(self) -> dict:
        if self._mock:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        try:
            state = self._client.GetState()
            return {
                'x': float(state.position[0]),
                'y': float(state.position[1]),
                'z': float(state.position[2]),
                'yaw': float(state.imu_state.rpy[2]),
            }
        except Exception as e:
            logger.error(f'get_pose() failed: {e}')
            return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

    def get_battery(self) -> float:
        if self._mock:
            return 100.0
        try:
            state = self._client.GetState()
            return float(state.bms_state.soc)
        except Exception as e:
            logger.error(f'get_battery() failed: {e}')
            return -1.0

    def get_state(self) -> str:
        return self._state

    def resume(self):
        self._state = 'idle'
        if self._mock:
            logger.info('[MOCK] resume()')
