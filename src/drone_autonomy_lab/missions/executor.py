# AP_FLAKE8_CLEAN
"""Execute explicit vehicle actions in order."""
from .models import MissionState, MissionStep


class MissionExecutor:
    """Advance only after a vehicle method confirms completion or raises."""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.state = MissionState.READY
        self.current_step = None
        self.completed_steps = []
        self.error = None

    def run(self, steps):
        if self.state is not MissionState.READY:
            raise RuntimeError('Use a new executor for each mission; missions are not automatically retried')
        steps = tuple(steps)
        if not steps or any(not isinstance(step, MissionStep) for step in steps):
            raise ValueError('Mission must contain one or more MissionStep objects')
        # Resolve every supported action before any command is sent.
        actions = [getattr(self.vehicle, step.action) for step in steps]
        if any(not callable(action) for action in actions):
            raise ValueError('Vehicle actions must be callable')

        self.state = MissionState.RUNNING
        try:
            for index, (step, action) in enumerate(zip(steps, actions), start=1):
                self.current_step = step
                print(f'Mission step {index}/{len(steps)}: {step.action} {dict(step.parameters)}')
                action(**step.parameters)
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
