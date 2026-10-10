# AP_FLAKE8_CLEAN
import unittest
from unittest.mock import Mock, patch

from pymavlink import mavutil

from drone_autonomy_lab.checks import check_altitude
from drone_autonomy_lab.vehicle import Drone


class TakeoffTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = Drone.__new__(Drone)
        self.vehicle.drone = Mock(target_system=1, target_component=1)
        self.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

    def message(self, **fields):
        message = Mock(**fields)
        message.get_srcSystem.return_value = 1
        message.get_srcComponent.return_value = 1
        return message

    def ack(self, result=0):
        return self.message(command=self.command, result=result)

    def test_threshold_and_units(self):
        self.assertFalse(check_altitude(self.message(relative_alt=9499), 10, 0.5))
        self.assertTrue(check_altitude(self.message(relative_alt=9500), 10, 0.5))
        self.assertTrue(check_altitude(self.message(relative_alt=10100), 10, 0.5))

    def test_acceptance_then_climb(self):
        foreign = self.message(relative_alt=10000)
        foreign.get_srcSystem.return_value = 2
        self.vehicle.drone.recv_match.side_effect = [
            self.message(command=400, result=0),
            self.ack(mavutil.mavlink.MAV_RESULT_IN_PROGRESS), self.ack(),
            None, foreign, self.message(relative_alt=5000), self.message(relative_alt=9500),
        ]
        self.vehicle.takeoff(10)
        self.vehicle.drone.mav.command_long_send.assert_called_once_with(
            1, 1, self.command, 0, 0, 0, 0, 0, 0, 0, 10,
        )
        self.assertEqual(self.vehicle.drone.recv_match.call_args.kwargs['type'], 'GLOBAL_POSITION_INT')

    def test_rejected_command(self):
        self.vehicle.drone.recv_match.return_value = self.ack(mavutil.mavlink.MAV_RESULT_DENIED)
        with self.assertRaisesRegex(RuntimeError, 'DENIED'):
            self.vehicle.takeoff(10)
        self.assertEqual(self.vehicle.drone.recv_match.call_count, 1)

    def test_missing_ack(self):
        self.vehicle.drone.recv_match.return_value = None
        with patch('drone_autonomy_lab.vehicle.time.monotonic', side_effect=[0, 0, 1, 31]):
            with self.assertRaisesRegex(TimeoutError, 'takeoff acknowledgment'):
                self.vehicle.takeoff(10)

    def test_ack_and_altitude_share_deadline(self):
        self.vehicle.drone.recv_match.side_effect = [self.ack(), self.message(relative_alt=1000)]
        with patch('drone_autonomy_lab.vehicle.time.monotonic', side_effect=[0, 0, 20, 20, 21, 31]):
            with self.assertRaisesRegex(TimeoutError, 'takeoff altitude'):
                self.vehicle.takeoff(10)

    def test_invalid_arguments_do_not_send(self):
        for kwargs in [
            {'altitude_m': 0}, {'altitude_m': -1}, {'altitude_m': float('nan')},
            {'altitude_m': float('inf')}, {'altitude_m': 10, 'tolerance_m': -1},
            {'altitude_m': 10, 'tolerance_m': 10}, {'altitude_m': 10, 'tolerance_m': float('nan')},
            {'altitude_m': 10, 'timeout': 0},
        ]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.vehicle.takeoff(**kwargs)
        self.vehicle.drone.mav.command_long_send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
