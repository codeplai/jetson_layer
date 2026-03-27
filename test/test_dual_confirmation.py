#!/usr/bin/env python3
"""Tests for Go2 Bridge Node — Dual Confirmation Architecture."""

import time
import threading
import pytest
from unittest.mock import MagicMock, patch


class Go2InterfaceMock:
    """Mock that logs actions instead of moving the real robot."""

    def __init__(self):
        self.actions = []
        self._state = 'idle'
        self.is_mock = True

    def stop(self) -> bool:
        self.actions.append('stop')
        self._state = 'stopped'
        return True

    def retreat(self, distance_m: float = 2.0) -> bool:
        self.actions.append(f'retreat({distance_m})')
        self._state = 'stopped'
        return True

    def set_velocity(self, linear_x: float, angular_z: float) -> bool:
        if self._state in ('stopped', 'retreating'):
            self.actions.append(f'set_velocity_BLOCKED(state={self._state})')
            return False
        self.actions.append(f'set_velocity({linear_x:.2f}, {angular_z:.2f})')
        self._state = 'moving'
        return True

    def scan_zone(self) -> dict:
        self.actions.append('scan_zone')
        self._state = 'idle'
        return {i * 45: {'clear': True} for i in range(8)}

    def get_pose(self) -> dict:
        return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

    def get_battery(self) -> float:
        return 85.0

    def get_state(self) -> str:
        return self._state

    def resume(self):
        self.actions.append('resume')
        self._state = 'idle'


class DualConfirmationEngine:
    """Standalone test harness for the dual confirmation logic."""

    def __init__(self, go2, timeout=2.0):
        self.go2 = go2
        self.timeout = timeout
        self._lock = threading.Lock()
        self.gas_status = 'SAFE'
        self.gas_ts = 0.0
        self.camera_obstacle = False
        self.camera_ts = 0.0
        self.lidar_obstacle = False
        self.lidar_ts = 0.0
        self.behavior = 'resume'

    def set_gas(self, status):
        with self._lock:
            self.gas_status = status
            self.gas_ts = time.time()

    def set_camera(self, obstacle):
        with self._lock:
            self.camera_obstacle = obstacle
            self.camera_ts = time.time()

    def set_lidar(self, obstacle):
        with self._lock:
            self.lidar_obstacle = obstacle
            self.lidar_ts = time.time()

    def decide(self):
        with self._lock:
            gas = self.gas_status
            gas_ts = self.gas_ts
            cam = self.camera_obstacle
            cam_ts = self.camera_ts
            lidar = self.lidar_obstacle
            lidar_ts = self.lidar_ts

        now = time.time()
        gas_alert = gas in ('WARNING', 'CRITICAL')
        gas_fresh = (now - gas_ts) < self.timeout if gas_ts > 0 else False

        vision_confirmed = False
        if cam and (now - cam_ts) < self.timeout:
            vision_confirmed = True
        if lidar and (now - lidar_ts) < self.timeout:
            vision_confirmed = True

        confirmed = gas_fresh and gas_alert and vision_confirmed

        if gas == 'CRITICAL' and confirmed:
            if self.behavior != 'stop':
                self.behavior = 'stop'
                self.go2.stop()
                self.go2.retreat(2.0)
        elif gas == 'WARNING' and confirmed:
            if self.behavior != 'slow':
                self.behavior = 'slow'
                self.go2.set_velocity(0.2, 0.0)
        elif gas == 'SAFE':
            if self.behavior != 'resume':
                self.behavior = 'resume'
                self.go2.resume()
                self.go2.set_velocity(0.4, 0.0)


# ── Tests ─────────────────────────────────────────────────────

class TestDualConfirmation:

    def _make_engine(self):
        mock = Go2InterfaceMock()
        engine = DualConfirmationEngine(mock, timeout=2.0)
        return engine, mock

    def test_gas_warning_without_confirmation_does_not_act(self):
        """Gas WARNING alone (no camera/lidar) should NOT trigger action."""
        engine, mock = self._make_engine()
        engine.set_gas('WARNING')
        engine.decide()
        assert mock.actions == []
        assert engine.behavior == 'resume'

    def test_gas_critical_plus_camera_triggers_stop_and_retreat(self):
        """Gas CRITICAL + camera detection = confirmed → stop + retreat."""
        engine, mock = self._make_engine()
        engine.set_gas('CRITICAL')
        engine.set_camera(True)
        engine.decide()
        assert 'stop' in mock.actions
        assert 'retreat(2.0)' in mock.actions
        assert engine.behavior == 'stop'

    def test_gas_critical_plus_lidar_triggers_stop_and_retreat(self):
        """Gas CRITICAL + lidar obstacle = confirmed → stop + retreat."""
        engine, mock = self._make_engine()
        engine.set_gas('CRITICAL')
        engine.set_lidar(True)
        engine.decide()
        assert 'stop' in mock.actions
        assert 'retreat(2.0)' in mock.actions

    def test_only_lidar_obstacle_without_gas_does_not_act(self):
        """LiDAR obstacle alone (gas SAFE) should NOT trigger action."""
        engine, mock = self._make_engine()
        engine.set_lidar(True)
        engine.decide()
        assert mock.actions == []

    def test_timeout_expires_no_confirmation(self):
        """Gas CRITICAL but camera data is stale (>2s) → no action."""
        engine, mock = self._make_engine()
        engine.set_camera(True)
        engine.camera_ts = time.time() - 3.0  # 3s ago, beyond 2s timeout
        engine.set_gas('CRITICAL')
        engine.decide()
        assert mock.actions == []

    def test_cmd_vel_blocked_when_stopped(self):
        """set_velocity should be blocked when robot state is 'stopped'."""
        mock = Go2InterfaceMock()
        mock.stop()  # state → stopped
        result = mock.set_velocity(0.3, 0.0)
        assert result is False
        assert 'set_velocity_BLOCKED(state=stopped)' in mock.actions

    def test_warning_plus_camera_reduces_speed(self):
        """Gas WARNING + camera confirmed → reduce speed."""
        engine, mock = self._make_engine()
        engine.set_gas('WARNING')
        engine.set_camera(True)
        engine.decide()
        assert engine.behavior == 'slow'
        assert 'set_velocity(0.20, 0.00)' in mock.actions

    def test_safe_after_warning_resumes(self):
        """Transition from WARNING confirmed → SAFE should resume."""
        engine, mock = self._make_engine()
        # First trigger warning
        engine.set_gas('WARNING')
        engine.set_camera(True)
        engine.decide()
        assert engine.behavior == 'slow'

        # Then gas clears
        mock.actions.clear()
        mock._state = 'idle'
        engine.set_gas('SAFE')
        engine.set_camera(False)
        engine.decide()
        assert engine.behavior == 'resume'
        assert 'resume' in mock.actions
