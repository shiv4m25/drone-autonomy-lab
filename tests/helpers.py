# AP_FLAKE8_CLEAN
"""Synthetic MAVLink messages and a queue-backed connection for receiver tests."""
import queue
from unittest.mock import Mock

from pymavlink import mavutil


def heartbeat(mode=4, armed=False, system=1, component=1, autopilot=None, vehicle_type=None):
    flags = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        flags |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    message = mavutil.mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_QUADROTOR if vehicle_type is None else vehicle_type,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA if autopilot is None else autopilot,
        flags, mode, mavutil.mavlink.MAV_STATE_ACTIVE, 3,
    )
    message._header.srcSystem = system
    message._header.srcComponent = component
    return message


def message(name, system=1, component=1, **fields):
    value = Mock(**fields)
    value.get_type.return_value = name
    value.get_srcSystem.return_value = system
    value.get_srcComponent.return_value = component
    return value


def ack(command=22, result=0, **source):
    return message('COMMAND_ACK', command=command, result=result, **source)


class QueuedConnection:
    def __init__(self):
        self.incoming = queue.Queue()
        self.mav = Mock()
        self.target_system = self.target_component = 1
        self.set_mode = Mock()
        self.mode_mapping = Mock(return_value={'GUIDED': 4, 'LAND': 9})
        self.close = Mock()

    def recv_match(self, *, blocking, timeout):
        try:
            return self.incoming.get(timeout=timeout)
        except queue.Empty:
            return None
