# AP_FLAKE8_CLEAN
import time
import unittest
from unittest.mock import Mock, patch

from pymavlink import mavutil

from drone_autonomy_lab.checks import check_altitude
from drone_autonomy_lab.missions.executor import MissionExecutor
from drone_autonomy_lab.missions.models import MissionState, MissionStep
from drone_autonomy_lab.telemetry.receiver import TelemetryReceiver
from drone_autonomy_lab.vehicle import Drone
from helpers import ack, heartbeat, message


class TakeoffTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = Drone.__new__(Drone)
        self.vehicle.drone = Mock(target_system=1, target_component=1)
        self.receiver = TelemetryReceiver(self.vehicle.drone)
        self.receiver._record(heartbeat(armed=True), time.monotonic())
        self.vehicle.telemetry = self.receiver
        self.executor = MissionExecutor(self.vehicle)
        self.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

    def run_takeoff(self, **parameters):
        return self.executor.run([MissionStep('takeoff', {'altitude_m': 10, **parameters})])

    def emit(self, *messages):
        for value in messages:
            self.receiver._record(value, time.monotonic())

    def test_threshold_and_units(self):
        self.assertFalse(check_altitude(message('GLOBAL_POSITION_INT', relative_alt=9499), 10, 0.5))
        self.assertTrue(check_altitude(message('GLOBAL_POSITION_INT', relative_alt=9500), 10, 0.5))
        self.assertTrue(check_altitude(message('GLOBAL_POSITION_INT', relative_alt=10100), 10, 0.5))

    def test_acceptance_then_climb(self):
        self.vehicle.drone.mav.command_long_send.side_effect = lambda *args: self.emit(
            ack(400), ack(result=mavutil.mavlink.MAV_RESULT_IN_PROGRESS), ack(),
            message('GLOBAL_POSITION_INT', system=2, relative_alt=10000),
            message('GLOBAL_POSITION_INT', relative_alt=5000),
            message('GLOBAL_POSITION_INT', relative_alt=9500),
        )
        self.assertEqual(self.run_takeoff(), MissionState.COMPLETED)
        self.vehicle.drone.mav.command_long_send.assert_called_once_with(
            1, 1, self.command, 0, 0, 0, 0, 0, 0, 0, 10,
        )
        self.vehicle.drone.recv_match.assert_not_called()

    def test_observation_before_ack_is_retained(self):
        self.vehicle.drone.mav.command_long_send.side_effect = lambda *args: self.emit(
            message('GLOBAL_POSITION_INT', relative_alt=9500), ack(),
        )
        self.assertEqual(self.run_takeoff(), MissionState.COMPLETED)

    def test_rejected_command(self):
        self.vehicle.drone.mav.command_long_send.side_effect = lambda *args: self.emit(
            ack(result=mavutil.mavlink.MAV_RESULT_DENIED),
        )
        with self.assertRaisesRegex(RuntimeError, 'DENIED'):
            self.run_takeoff()
        self.assertEqual(self.executor.completed_steps, [])
        self.assertEqual(self.executor.state, MissionState.FAILED)

    def test_missing_ack(self):
        with self.assertRaisesRegex(TimeoutError, 'takeoff acknowledgment'):
            self.run_takeoff(timeout=0.01)

    def test_cached_ack_and_altitude_cannot_confirm_new_takeoff(self):
        self.emit(ack(), message('GLOBAL_POSITION_INT', relative_alt=10000))
        with self.assertRaisesRegex(TimeoutError, 'takeoff acknowledgment'):
            self.run_takeoff(timeout=0.01)

    def test_ack_and_altitude_share_deadline(self):
        telemetry = Mock()
        telemetry.wait_for.return_value = message('GLOBAL_POSITION_INT', relative_alt=10000)
        executor = MissionExecutor(self.vehicle, telemetry)
        with patch('drone_autonomy_lab.missions.executor.time.monotonic', return_value=100):
            executor.run([MissionStep('takeoff', {'altitude_m': 10, 'timeout': 30})])
        waits = telemetry.wait_for.call_args_list
        self.assertEqual([item.kwargs['deadline'] for item in waits], [130, 130])
        self.assertEqual([item.kwargs['after'] for item in waits], [telemetry.mark.return_value] * 2)
        # A late accepted ACK consumes the same deadline, rather than starting another wait.
        self.vehicle.drone.mav.command_long_send.side_effect = lambda *args: self.emit(ack())
        with self.assertRaisesRegex(TimeoutError, 'takeoff altitude'):
            self.run_takeoff(timeout=0.01)

    def test_invalid_arguments_do_not_send(self):
        for kwargs in [
            {'altitude_m': 0}, {'altitude_m': -1}, {'altitude_m': float('nan')},
            {'altitude_m': float('inf')}, {'altitude_m': 10, 'tolerance_m': -1},
            {'altitude_m': 10, 'tolerance_m': 10}, {'altitude_m': 10, 'tolerance_m': float('nan')},
            {'altitude_m': 10, 'timeout': 0},
        ]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.executor.run([MissionStep('takeoff', kwargs)])
        self.vehicle.drone.mav.command_long_send.assert_not_called()
        for altitude in (False, 0, -1, float('inf'), float('nan')):
            with self.subTest(altitude=altitude), self.assertRaises(ValueError):
                self.vehicle.takeoff(altitude)
        self.vehicle.drone.mav.command_long_send.assert_not_called()
