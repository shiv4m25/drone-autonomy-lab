# AP_FLAKE8_CLEAN
"""Send ordered actions and confirm their results using received telemetry."""
import time

from pymavlink import mavutil

from ..checks import check_ack, check_altitude, check_armed, check_mode
from .models import MissionState, MissionStep


DEFAULT_TIMEOUTS = {'set_mode': 10, 'arm': 10, 'takeoff': 30, 'land': 90}


class MissionExecutor:
    """Advance only after ACK/state confirmation, or stop on an error."""

    def __init__(self, vehicle, telemetry=None):
        self.vehicle = vehicle
        self.telemetry = vehicle.telemetry if telemetry is None else telemetry
        self.state = MissionState.READY
        self.current_step = None
        self.completed_steps = []
        self.error = None

    def _confirm(self, step, after, deadline):
        def wait(message_type, check, description):
            return self.telemetry.wait_for(
                message_type, check, after=after, deadline=deadline, description=description,
            )

        if step.action == 'set_mode':
            mode = step.parameters['mode']
            wait('HEARTBEAT', lambda message: check_mode(message, mode), f'{mode} mode')
        elif step.action == 'arm':
            command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            wait('COMMAND_ACK', lambda message: check_ack(message, command), 'arm acknowledgment')
            wait('HEARTBEAT', check_armed, 'armed state')
        elif step.action == 'takeoff':
            command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            altitude = step.parameters['altitude_m']
            tolerance = step.parameters.get('tolerance_m', 0.5)
            wait('COMMAND_ACK', lambda message: check_ack(message, command), 'takeoff acknowledgment')
            print('Takeoff accepted; waiting for altitude...')
            position = wait(
                'GLOBAL_POSITION_INT', lambda message: check_altitude(message, altitude, tolerance),
                f'takeoff altitude of {altitude:g} m',
            )
            print(f'Confirmed: altitude relative to home is {position.relative_alt / 1000:.2f} m')
        elif step.action == 'land':
            wait(
                'HEARTBEAT', lambda message: check_mode(message, 'LAND') and not check_armed(message),
                'LAND mode and disarmed state',
            )

    def run(self, steps):
        if self.state is not MissionState.READY:
            raise RuntimeError('Use a new executor for each mission; missions are not automatically retried')
        steps = tuple(steps)
        if not steps or any(not isinstance(step, MissionStep) for step in steps):
            raise ValueError('Mission must contain one or more MissionStep objects')
        actions = [getattr(self.vehicle, step.action) for step in steps]
        if any(not callable(action) for action in actions):
            raise ValueError('Vehicle actions must be callable')

        self.state = MissionState.RUNNING
        try:
            for index, (step, action) in enumerate(zip(steps, actions), start=1):
                self.current_step = step
                print(f'Mission step {index}/{len(steps)}: {step.action} {dict(step.parameters)}')
                timeout = step.parameters.get('timeout', DEFAULT_TIMEOUTS[step.action])
                deadline = time.monotonic() + timeout
                after = self.telemetry.mark()
                send_parameters = {key: value for key, value in step.parameters.items()
                                   if key not in ('timeout', 'tolerance_m')}
                action(**send_parameters)
                self._confirm(step, after, deadline)
                self.completed_steps.append(step)
        except KeyboardInterrupt as error:
            self.state = MissionState.INTERRUPTED
            self.error = error
            print(f'Mission interrupted during {self.current_step.action}')
            raise
        except Exception as error:
            self.state = MissionState.FAILED
            self.error = error
            print(f'Mission failed during {self.current_step.action}: {error}')
            raise
        self.current_step = None
        self.state = MissionState.COMPLETED
        print('Mission completed')
        return self.state
