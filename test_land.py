# AP_FLAKE8_CLEAN
import unittest
from unittest.mock import Mock, call, patch

from pymavlink import mavutil

from drone_connect import Drone, main


class LandTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = Drone.__new__(Drone)
        self.vehicle.drone = Mock(target_system=1, target_component=1)

    def heartbeat(self, mode=9, armed=True, system=1, component=1):
        flags = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if armed:
            flags |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        message = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            flags, mode, mavutil.mavlink.MAV_STATE_ACTIVE, 3,
        )
        message._header.srcSystem = system
        message._header.srcComponent = component
        return message

    def test_waits_for_land_and_disarmed_from_target(self):
        self.vehicle.drone.recv_match.side_effect = [
            None, self.heartbeat(mode=4, armed=False), self.heartbeat(),
            self.heartbeat(armed=False, system=2),
            self.heartbeat(armed=False, component=2), self.heartbeat(armed=False),
        ]
        self.vehicle.land()
        self.vehicle.drone.set_mode.assert_called_once_with('LAND')
        self.assertEqual(self.vehicle.drone.recv_match.call_count, 6)
        self.vehicle.drone.mav.command_long_send.assert_not_called()

    def test_timeout_if_still_armed_or_missing_telemetry(self):
        for message in [self.heartbeat(), None]:
            with self.subTest(message=message):
                self.vehicle.drone.recv_match.return_value = message
                with patch('drone_connect.time.monotonic', side_effect=[0, 0, 1, 91]):
                    with self.assertRaisesRegex(TimeoutError, 'LAND mode and disarmed state'):
                        self.vehicle.land()

    def test_invalid_timeout_does_not_send(self):
        for timeout in [0, -1, float('inf'), float('nan')]:
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                self.vehicle.land(timeout)
        self.vehicle.drone.set_mode.assert_not_called()

    def test_mission_order_and_cleanup(self):
        with patch('drone_connect.Drone') as factory:
            main()
            self.assertEqual(factory.return_value.method_calls, [
                call.set_mode('GUIDED'), call.arm(), call.takeoff(10), call.land(), call.close(),
            ])

    def test_landing_failure_still_closes_connection(self):
        with patch('drone_connect.Drone') as factory:
            factory.return_value.land.side_effect = TimeoutError('landing timeout')
            with self.assertRaises(TimeoutError):
                main()
            factory.return_value.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
