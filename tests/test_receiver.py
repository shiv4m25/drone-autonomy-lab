# AP_FLAKE8_CLEAN
import time
import unittest
from unittest.mock import Mock, patch

from pymavlink import mavutil

from drone_autonomy_lab.missions.executor import MissionExecutor
from drone_autonomy_lab.missions.demo import takeoff_land_mission
from drone_autonomy_lab.missions.models import MissionState
from drone_autonomy_lab.telemetry.receiver import TelemetryReceiver
from drone_autonomy_lab.vehicle import Drone
from helpers import QueuedConnection, ack, heartbeat, message


class ReceiverTest(unittest.TestCase):
    def setUp(self):
        self.connection = QueuedConnection()
        self.receiver = TelemetryReceiver(self.connection)
        self.addCleanup(self.receiver.close)

    def wait(self, name, check=lambda value: True, after=0, timeout=0.05):
        return self.receiver.wait_for(name, check, after=after, deadline=time.monotonic() + timeout, description=name)

    def record(self, *messages):
        for value in messages:
            self.receiver._record(value, time.monotonic())

    def test_background_reader_keeps_receiving_and_sends_nothing(self):
        self.receiver.start()
        self.connection.incoming.put(heartbeat())
        self.wait('HEARTBEAT', timeout=1)
        mark = self.receiver.mark()
        value = message('ATTITUDE', roll=0, pitch=0, yaw=0)
        self.connection.incoming.put(value)
        self.assertIs(self.wait('ATTITUDE', after=mark, timeout=1), value)
        self.assertIn('Attitude roll/pitch/yaw: 0.0', self.receiver.render(time.monotonic(), 5))
        self.connection.mav.command_long_send.assert_not_called()
        self.connection.set_mode.assert_not_called()
        self.receiver.close()
        self.assertFalse(self.receiver._thread.is_alive())
        self.connection.close.assert_not_called()

    def test_source_discovery_and_filtering(self):
        self.record(
            ack(), heartbeat(autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID),
            heartbeat(vehicle_type=mavutil.mavlink.MAV_TYPE_GCS),
        )
        self.assertIsNone(self.receiver.source)
        self.record(heartbeat())
        mark = self.receiver.mark()
        self.record(ack(system=2), ack(component=2))
        with self.assertRaises(TimeoutError):
            self.wait('COMMAND_ACK', after=mark, timeout=0.01)
        good = ack()
        self.record(good)
        self.assertIs(self.wait('COMMAND_ACK', after=mark), good)

    def test_stale_post_mark_observations_are_not_confirmation(self):
        self.record(heartbeat())
        mark = self.receiver.mark()
        self.receiver._record(ack(), time.monotonic() - 6)
        with self.assertRaises(TimeoutError):
            self.wait('COMMAND_ACK', after=mark, timeout=0.01)

    def test_receiver_failure_is_propagated_with_original_cause(self):
        cause = OSError('link closed')
        self.receiver.connection = Mock()
        self.receiver.connection.recv_match.side_effect = cause
        self.receiver.start()
        with self.assertRaisesRegex(RuntimeError, 'receiver failed') as caught:
            self.wait('HEARTBEAT', timeout=1)
        self.assertIs(caught.exception.__cause__, cause)

    def test_stop_wakes_confirmation_and_disallows_restart(self):
        self.receiver.close()
        with self.assertRaisesRegex(RuntimeError, 'stopped'):
            self.wait('HEARTBEAT')
        with self.assertRaises(RuntimeError):
            self.receiver.start()

    def test_expired_deadline_and_history_overflow_fail_explicitly(self):
        self.record(heartbeat())
        with self.assertRaises(TimeoutError):
            self.receiver.wait_for('HEARTBEAT', lambda value: True, after=0,
                                   deadline=time.monotonic() - 1, description='expired')
        for _ in range(2048):
            self.record(ack())
        with self.assertRaisesRegex(RuntimeError, 'overflowed'):
            self.wait('COMMAND_ACK')

    def test_full_mission_with_background_receiver(self):
        self.connection.incoming.put(heartbeat(mode=0))

        def change_mode(mode):
            self.connection.incoming.put(heartbeat(mode=4 if mode == 'GUIDED' else 9, armed=False))

        def send_command(system, component, command, *parameters):
            self.connection.incoming.put(ack(command))
            if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                self.connection.incoming.put(heartbeat(armed=True))
            elif command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                self.connection.incoming.put(message('ATTITUDE', roll=0, pitch=0, yaw=0))
                self.connection.incoming.put(message(
                    'GLOBAL_POSITION_INT', relative_alt=9500, lat=0, lon=0, alt=9500, vx=0, vy=0, vz=0,
                ))
        self.connection.set_mode.side_effect = change_mode
        self.connection.mav.command_long_send.side_effect = send_command
        with patch('drone_autonomy_lab.vehicle.mavutil.mavlink_connection', return_value=self.connection):
            vehicle = Drone()
        try:
            executor = MissionExecutor(vehicle)
            self.assertEqual(executor.run(takeoff_land_mission()), MissionState.COMPLETED)
            self.assertEqual(len(executor.completed_steps), 4)
            self.assertEqual(self.connection.mav.command_long_send.call_count, 2)
            self.assertIn('Attitude roll/pitch/yaw: 0.0', vehicle.telemetry.render(time.monotonic(), 5))
        finally:
            vehicle.close()
        self.connection.close.assert_called_once()
        self.assertFalse(vehicle.telemetry._thread.is_alive())

    def test_vehicle_closes_connection_if_initial_heartbeat_wait_fails(self):
        with (
            patch('drone_autonomy_lab.vehicle.mavutil.mavlink_connection', return_value=self.connection),
            patch('drone_autonomy_lab.vehicle.TelemetryReceiver') as factory,
        ):
            error = TimeoutError('autopilot heartbeat')
            factory.return_value.wait_for.side_effect = error
            with self.assertRaises(TimeoutError) as caught:
                Drone()
        self.assertIs(caught.exception, error)
        factory.return_value.close.assert_called_once()
        self.connection.close.assert_called_once()
