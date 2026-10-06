# AP_FLAKE8_CLEAN
from pymavlink import mavutil


def check_mode(heartbeat, mode):
    return mavutil.mode_string_v10(heartbeat) == mode


def check_armed(heartbeat):
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def check_altitude(position, altitude_m, tolerance_m):
    return position.relative_alt / 1000.0 >= altitude_m - tolerance_m


def check_ack(ack, command):
    if ack.command != command:
        return False
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        return True
    if ack.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
        return False
    result = mavutil.mavlink.enums['MAV_RESULT'].get(ack.result)
    name = result.name if result is not None else str(ack.result)
    raise RuntimeError(f"Command {command} failed: {name}")
