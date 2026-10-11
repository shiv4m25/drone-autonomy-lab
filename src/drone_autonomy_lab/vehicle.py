# AP_FLAKE8_CLEAN
"""Manage the connection and send explicit ArduPilot vehicle inputs."""
import math
import time

from pymavlink import mavutil

from .telemetry.receiver import TelemetryReceiver


class Drone:
    """Action methods send requests; MissionExecutor owns confirmation."""

    def __init__(self, connection='udpin:127.0.0.1:14560'):
        self.drone = mavutil.mavlink_connection(connection)
        self.telemetry = TelemetryReceiver(self.drone)
        try:
            self.telemetry.start()
            print('Waiting for simulated drone')
            heartbeat = self.telemetry.wait_for(
                'HEARTBEAT', lambda message: True, after=0,
                deadline=time.monotonic() + 15, description='autopilot heartbeat',
            )
            self.drone.target_system = heartbeat.get_srcSystem()
            self.drone.target_component = heartbeat.get_srcComponent()
            print(f'Connected to drone {self.drone.target_system}')
            print(f'Flight mode: {mavutil.mode_string_v10(heartbeat)}')
        except BaseException:
            self.close()
            raise

    def set_mode(self, mode):
        mode = mode.upper()
        if mode not in self.drone.mode_mapping():
            raise ValueError(f'Unsupported flight mode: {mode}')
        self.drone.set_mode(mode)

    def arm(self):
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0,
        )

    def takeoff(self, altitude_m):
        if isinstance(altitude_m, bool) or not math.isfinite(altitude_m) or altitude_m <= 0:
            raise ValueError('Takeoff altitude must be finite and positive')
        self.drone.mav.command_long_send(
            self.drone.target_system, self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,                    # First transmission
            0, 0, 0, 0, 0, 0,    # Parameters 1–6
            altitude_m,           # Parameter 7: altitude relative to home in metres
        )

    def land(self):
        self.set_mode('LAND')

    def close(self):
        try:
            self.telemetry.close()
        finally:
            self.drone.close()
