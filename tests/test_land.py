# AP_FLAKE8_CLEAN
import time
import unittest
from unittest.mock import Mock, call, patch

from drone_autonomy_lab.missions.demo import main
from drone_autonomy_lab.missions.executor import MissionExecutor
from drone_autonomy_lab.missions.models import MissionStep
from drone_autonomy_lab.telemetry.receiver import TelemetryReceiver
from drone_autonomy_lab.vehicle import Drone
from helpers import heartbeat


class LandTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = Drone.__new__(Drone)
        self.vehicle.drone = Mock(target_system=1, target_component=1)
        self.receiver = TelemetryReceiver(self.vehicle.drone)
        self.receiver._record(heartbeat(), time.monotonic())
        self.vehicle.telemetry = self.receiver
        self.vehicle.drone.mode_mapping.return_value = {'GUIDED': 4, 'LAND': 9}

    def run_land(self, timeout=90):
        return MissionExecutor(self.vehicle).run([MissionStep('land', {'timeout': timeout})])

    def test_waits_for_land_and_disarmed_from_target(self):
        def emit(mode):
            for value in [
                heartbeat(mode=4, armed=False), heartbeat(mode=9, armed=True),
                heartbeat(mode=9, system=2), heartbeat(mode=9, component=2), heartbeat(mode=9),
            ]:
                self.receiver._record(value, time.monotonic())
        self.vehicle.drone.set_mode.side_effect = emit
        self.run_land()
        self.vehicle.drone.set_mode.assert_called_once_with('LAND')
        self.vehicle.drone.mav.command_long_send.assert_not_called()
        self.vehicle.drone.recv_match.assert_not_called()

    def test_timeout_if_still_armed_or_missing_telemetry(self):
        for value in [heartbeat(mode=9, armed=True), None]:
            with self.subTest(message=value):
                def emit(mode):
                    if value is not None:
                        self.receiver._record(value, time.monotonic())
                self.vehicle.drone.set_mode.side_effect = emit
                with self.assertRaisesRegex(TimeoutError, 'LAND mode and disarmed state'):
                    self.run_land(timeout=0.01)

    def test_invalid_timeout_does_not_send(self):
        for timeout in [0, -1, float('inf'), float('nan')]:
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                self.run_land(timeout)
        self.vehicle.drone.set_mode.assert_not_called()

    def test_mission_order_and_cleanup(self):
        with patch('drone_autonomy_lab.missions.demo.Drone') as factory:
            factory.return_value.telemetry.wait_for.return_value.relative_alt = 10000
            main([])
            calls = [item for item in factory.return_value.method_calls if not item[0].startswith('telemetry.')]
            self.assertEqual(calls, [
                call.set_mode(mode='GUIDED'), call.arm(), call.takeoff(altitude_m=10), call.land(), call.close(),
            ])

    def test_landing_failure_still_closes_connection(self):
        with patch('drone_autonomy_lab.missions.demo.Drone') as factory:
            factory.return_value.telemetry.wait_for.return_value.relative_alt = 10000

            def wait(message_type, check, **kwargs):
                if kwargs['description'] == 'LAND mode and disarmed state':
                    raise TimeoutError('landing timeout')
                return Mock(relative_alt=10000)
            factory.return_value.telemetry.wait_for.side_effect = wait
            with self.assertRaises(TimeoutError):
                main([])
            factory.return_value.close.assert_called_once()
