# AP_FLAKE8_CLEAN
"""Latest autopilot telemetry, unit conversion, and reception freshness."""
import math

from pymavlink import mavutil

from ..checks import check_armed


MESSAGE_TYPES = {
    'HEARTBEAT', 'GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED', 'ATTITUDE',
    'BATTERY_STATUS', 'SYS_STATUS', 'GPS_RAW_INT', 'STATUSTEXT',
}


def measurement(value, unknown, scale, unit, precision=2):
    if value == unknown:
        return 'unknown'
    return f'{value / scale:.{precision}f} {unit}'


class TelemetryState:
    """Keep the latest message and local reception time for one autopilot."""

    def __init__(self):
        self.source = None
        self.messages = {}

    def update(self, message, received_at):
        name = message.get_type()
        if name not in MESSAGE_TYPES:
            return
        source = (message.get_srcSystem(), message.get_srcComponent())
        if self.source is None:
            if name != 'HEARTBEAT' or message.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                return
            if message.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self.source = source
        if source != self.source:
            return
        self.messages[name] = (message, received_at)

    def render(self, now, stale_after):
        lines = ['Drone telemetry — Ctrl+C to stop']
        if self.source is None:
            lines.append('Waiting for an autopilot heartbeat (check MAVProxy output port).')
        else:
            lines.append(f'System {self.source[0]}, component {self.source[1]}')

        def add(name, title, describe):
            entry = self.messages.get(name)
            if entry is None:
                lines.append(f'{title}: not received')
                return
            message, received_at = entry
            age = max(0, now - received_at)
            label = 'STALE' if age >= stale_after else 'fresh'
            lines.append(f'{title}: {describe(message)} [{label}, age {age:.1f}s]')

        add('HEARTBEAT', 'State', lambda m: (
            f'mode={mavutil.mode_string_v10(m)}, armed={"YES" if check_armed(m) else "NO"}, '
            f'system_status={m.system_status}'
        ))
        add('GLOBAL_POSITION_INT', 'Global position', lambda m: (
            f'lat={m.lat / 1e7:.7f}, lon={m.lon / 1e7:.7f}, '
            f'altitude(home)={m.relative_alt / 1000:.2f} m, altitude(MSL)={m.alt / 1000:.2f} m'
        ))
        add('GLOBAL_POSITION_INT', 'Global velocity N/E/D', lambda m: (
            f'{m.vx / 100:.2f} / {m.vy / 100:.2f} / {m.vz / 100:.2f} m/s'
        ))
        add('LOCAL_POSITION_NED', 'Local position N/E/D', lambda m: (
            f'{m.x:.2f} / {m.y:.2f} / {m.z:.2f} m (local origin)'
        ))
        add('LOCAL_POSITION_NED', 'Local velocity N/E/D', lambda m: (
            f'{m.vx:.2f} / {m.vy:.2f} / {m.vz:.2f} m/s'
        ))
        add('ATTITUDE', 'Attitude roll/pitch/yaw', lambda m: (
            f'{math.degrees(m.roll):.1f} / {math.degrees(m.pitch):.1f} / {math.degrees(m.yaw):.1f} deg'
        ))
        add('BATTERY_STATUS', 'Battery', self._battery)
        add('SYS_STATUS', 'Sensors', lambda m: (
            f'enabled but unhealthy mask=0x{m.onboard_control_sensors_enabled & ~m.onboard_control_sensors_health:x}, '
            f'link drop={m.drop_rate_comm / 100:.2f}%'
        ))
        add('GPS_RAW_INT', 'GPS', self._gps)
        add('STATUSTEXT', 'Last status text', self._status_text)
        return '\n'.join(lines)

    @staticmethod
    def _battery(message):
        cells = [voltage for voltage in message.voltages if voltage != 65535]
        cells += [voltage for voltage in getattr(message, 'voltages_ext', ()) if voltage not in (0, 65535)]
        voltage = f'{sum(cells) / 1000:.2f} V' if cells else 'unknown'
        current = measurement(message.current_battery, -1, 100, 'A')
        remaining = measurement(message.battery_remaining, -1, 1, '%', precision=0)
        used = measurement(message.current_consumed, -1, 1, 'mAh', precision=0)
        return f'id={message.id}, voltage={voltage}, current={current}, remaining={remaining}, used={used}'

    @staticmethod
    def _gps(message):
        satellites = message.satellites_visible if message.satellites_visible != 255 else 'unknown'
        hdop = f'{message.eph / 100:.2f}' if message.eph != 65535 else 'unknown'
        return f'fix_type={message.fix_type}, satellites={satellites}, HDOP={hdop}'

    @staticmethod
    def _status_text(message):
        text = message.text
        if isinstance(text, bytes):
            text = text.decode('utf-8', errors='replace')
        text = text.split('\x00', 1)[0]
        text = ''.join(char if char.isprintable() else ' ' for char in text)
        return f'severity={message.severity}, {text}'
