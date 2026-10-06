# AP_FLAKE8_CLEAN
import math
import time

from pymavlink import mavutil

from command_utils import check_ack, check_altitude, check_armed, check_mode


class Drone:
    def __init__(self):
        self.drone = mavutil.mavlink_connection("udpin:127.0.0.1:14560")
        try:
            self._establish_connection()
        except BaseException:
            self.close()
            raise

    def _establish_connection(self):

        print("Waiting for simulated drone")
        heartbeat = self._get_hb()
        self.drone.target_system = heartbeat.get_srcSystem()
        self.drone.target_component = heartbeat.get_srcComponent()
        print(f"Connected to drone {self.drone.target_system}")
        print(f"Flight mode: {mavutil.mode_string_v10(heartbeat)}")

    def _get_hb(self):
        hb = self.drone.wait_heartbeat(timeout=15)
        if hb is None:
            raise TimeoutError("No heartbeat received within 15 seconds")
        return hb

    @staticmethod
    def _deadline(timeout=10):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Timeout must be finite and positive")
        return time.monotonic() + timeout

    def _blocking_confirmation(self, message_type, check, deadline, description):
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Timed out waiting for {description}")
            message = self.drone.recv_match(
                type=message_type, blocking=True, timeout=min(1, remaining)
            )
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out waiting for {description}")
            if message is None:
                continue
            if (message.get_srcSystem() != self.drone.target_system
                    or message.get_srcComponent() != self.drone.target_component):
                continue
            if check(message):
                return message

    def set_mode(self, mode, timeout=10):
        mode = mode.upper()
        if mode not in self.drone.mode_mapping():
            raise ValueError(f"Unsupported flight mode: {mode}")
        deadline = self._deadline(timeout)
        self.drone.set_mode(mode)
        print(f"Requested {mode}; waiting for confirmation...")
        self._blocking_confirmation(
            "HEARTBEAT", lambda heartbeat: check_mode(heartbeat, mode),
            deadline, f"{mode} mode",
        )
        print(f"Confirmed: drone is in {mode} mode")

    def arm(self, timeout=10):
        deadline = self._deadline(timeout)
        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            command, 0, 1, 0, 0, 0, 0, 0, 0
        )
        print("Requested arming; waiting for acknowledgment...")
        self._blocking_confirmation(
            "COMMAND_ACK", lambda ack: check_ack(ack, command),
            deadline, "arm acknowledgment",
        )
        self._blocking_confirmation(
            "HEARTBEAT", check_armed, deadline, "armed state",
        )
        print("Confirmed: drone is armed")

    def takeoff(self, altitude_m, timeout=30, tolerance_m=0.5):
        """Wait for takeoff altitude relative to home, allowing a small shortfall."""
        if not math.isfinite(altitude_m) or altitude_m <= 0:
            raise ValueError("Takeoff altitude must be finite and positive")
        if not math.isfinite(tolerance_m) or not 0 <= tolerance_m < altitude_m:
            raise ValueError("Tolerance must be finite, nonnegative, and less than altitude")
        deadline = self._deadline(timeout)
        command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            command,
            0,                    # First transmission
            0, 0, 0, 0, 0, 0,    # Parameters 1–6
            altitude_m           # Parameter 7: altitude in metres
        )

        print(f"Requested takeoff to {altitude_m:g} m; waiting for acknowledgment...")
        self._blocking_confirmation(
            "COMMAND_ACK", lambda ack: check_ack(ack, command),
            deadline, "takeoff acknowledgment",
        )
        print("Takeoff accepted; waiting for altitude...")
        position = self._blocking_confirmation(
            "GLOBAL_POSITION_INT",
            lambda message: check_altitude(message, altitude_m, tolerance_m),
            deadline, f"takeoff altitude of {altitude_m:g} m",
        )
        print(f"Confirmed: altitude relative to home is {position.relative_alt / 1000.0:.2f} m")

    def land(self, timeout=90):
        """Request LAND and wait for automatic disarming in that mode."""
        deadline = self._deadline(timeout)
        self.drone.set_mode("LAND")
        print("Requested LAND; waiting for automatic disarming...")
        self._blocking_confirmation(
            "HEARTBEAT",
            lambda heartbeat: check_mode(heartbeat, "LAND") and not check_armed(heartbeat),
            deadline, "LAND mode and disarmed state",
        )
        print("Confirmed: drone is in LAND mode and disarmed")

    def close(self):
        self.drone.close()


def main():
    drone = Drone()
    try:
        drone.set_mode("GUIDED")
        drone.arm()
        drone.takeoff(10)
        drone.land()
    finally:
        drone.close()


if __name__ == "__main__":
    main()
