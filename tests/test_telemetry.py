# AP_FLAKE8_CLEAN
import argparse
import io
import math
import unittest
from contextlib import redirect_stdout
from unittest.mock import Mock, patch

from pymavlink import mavutil

from drone_autonomy_lab.telemetry.monitor import main, monitor, positive_number
from drone_autonomy_lab.telemetry.state import TelemetryState


class TelemetryTest(unittest.TestCase):
    def setUp(self):
        self.state = TelemetryState()
        self.state.update(self.heartbeat(), 10)

    def heartbeat(self, system=1, component=1, autopilot=None):
        autopilot = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA if autopilot is None else autopilot
        message = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_QUADROTOR, autopilot,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            4, mavutil.mavlink.MAV_STATE_ACTIVE, 3,
        )
        message._header.srcSystem = system
        message._header.srcComponent = component
        return message

    def message(self, name, **fields):
        message = Mock(**fields)
        message.get_type.return_value = name
        message.get_srcSystem.return_value = 1
        message.get_srcComponent.return_value = 1
        return message

    def test_discovers_autopilot_and_filters_other_sources(self):
        state = TelemetryState()
        state.update(self.message('GLOBAL_POSITION_INT'), 0)
        state.update(self.heartbeat(autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID), 0)
        self.assertIsNone(state.source)
        state.update(self.heartbeat(), 1)
        state.update(self.heartbeat(system=2), 2)
        state.update(self.heartbeat(component=2), 2)
        self.assertEqual(state.source, (1, 1))
        self.assertEqual(state.messages['HEARTBEAT'][1], 1)
        self.assertIn('mode=GUIDED, armed=YES', state.render(2, 5))

    def test_per_message_staleness(self):
        self.state.update(self.message('ATTITUDE', roll=0, pitch=0, yaw=0), 14)
        output = self.state.render(15, 5)
        self.assertIn('system_status=4 [STALE, age 5.0s]', output)
        self.assertIn('0.0 / 0.0 / 0.0 deg [fresh, age 1.0s]', output)
        self.assertIn('Global position: not received', output)

    def test_position_and_attitude_units(self):
        self.state.update(self.message(
            'GLOBAL_POSITION_INT', lat=123456789, lon=-234567890, relative_alt=10000,
            alt=123000, vx=150, vy=-200, vz=50,
        ), 10)
        self.state.update(self.message(
            'LOCAL_POSITION_NED', x=10, y=-2, z=-10, vx=1.5, vy=-2, vz=0.5,
        ), 10)
        self.state.update(self.message('ATTITUDE', roll=math.pi / 2, pitch=0, yaw=-math.pi), 10)
        output = self.state.render(10, 5)
        self.assertIn('lat=12.3456789, lon=-23.4567890', output)
        self.assertIn('altitude(home)=10.00 m, altitude(MSL)=123.00 m', output)
        self.assertIn('1.50 / -2.00 / 0.50 m/s', output)
        self.assertIn('10.00 / -2.00 / -10.00 m (local origin)', output)
        self.assertIn('90.0 / 0.0 / -180.0 deg', output)

    def test_battery_scaling_and_unknown_values(self):
        battery = self.message(
            'BATTERY_STATUS', id=0, voltages=[4200, 4100, 4100] + [65535] * 7,
            voltages_ext=[0, 0, 0, 0], current_battery=250, battery_remaining=96, current_consumed=100,
        )
        self.state.update(battery, 10)
        output = self.state.render(10, 5)
        self.assertIn('voltage=12.40 V, current=2.50 A, remaining=96 %, used=100 mAh', output)
        battery.voltages = [65535] * 10
        battery.current_battery = battery.battery_remaining = battery.current_consumed = -1
        self.assertIn('voltage=unknown, current=unknown, remaining=unknown, used=unknown', self.state.render(10, 5))

    def test_health_gps_and_terminal_safe_warning(self):
        self.state.update(self.message(
            'SYS_STATUS', onboard_control_sensors_enabled=7, onboard_control_sensors_health=5, drop_rate_comm=125,
        ), 10)
        self.state.update(self.message('GPS_RAW_INT', fix_type=3, satellites_visible=255, eph=65535), 10)
        self.state.update(self.message('STATUSTEXT', severity=4, text=b'Battery\x1b[2J\nwarning\x00padding'), 10)
        output = self.state.render(10, 5)
        self.assertIn('unhealthy mask=0x2, link drop=1.25%', output)
        self.assertIn('fix_type=3, satellites=unknown, HDOP=unknown', output)
        self.assertIn('severity=4, Battery [2J warning', output)
        self.assertNotIn('\x1b', output)
        self.assertNotIn('padding', output)

    def test_monitor_receives_without_sending(self):
        connection = Mock()
        with (
            patch('drone_autonomy_lab.telemetry.monitor.TelemetryReceiver') as factory,
            patch('drone_autonomy_lab.telemetry.monitor.time.monotonic', side_effect=[0, 0, 1, 2]),
            patch('drone_autonomy_lab.telemetry.monitor.time.sleep'),
            redirect_stdout(io.StringIO()) as output,
        ):
            receiver = factory.return_value.start.return_value
            receiver.render.side_effect = ['Waiting for an autopilot heartbeat', 'mode=GUIDED']
            monitor(connection, duration=2)
        self.assertIn('Waiting for an autopilot heartbeat', output.getvalue())
        self.assertIn('mode=GUIDED', output.getvalue())
        factory.assert_called_once_with(connection)
        receiver.close.assert_called_once()
        connection.recv_match.assert_not_called()
        self.assertEqual(connection.mav.method_calls, [])
        connection.set_mode.assert_not_called()

    def test_cleanup_on_ctrl_c(self):
        with (
            patch('drone_autonomy_lab.telemetry.monitor.mavutil.mavlink_connection') as factory,
            patch('drone_autonomy_lab.telemetry.monitor.monitor') as receive,
        ):
            receive.side_effect = KeyboardInterrupt
            with patch('sys.argv', ['drone-telemetry']), redirect_stdout(io.StringIO()):
                main()
            factory.assert_called_once_with('udpin:127.0.0.1:14561')
            factory.return_value.close.assert_called_once()

    def test_invalid_intervals(self):
        for value in ['0', '-1', 'nan', 'inf']:
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                positive_number(value)
        self.assertEqual(positive_number('0.5'), 0.5)


if __name__ == '__main__':
    unittest.main()
