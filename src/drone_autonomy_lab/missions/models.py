# AP_FLAKE8_CLEAN
"""Validated inputs and lifecycle states for explicit mission steps."""
import math
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Mapping


class MissionState(Enum):
    READY = 'ready'
    RUNNING = 'running'
    COMPLETED = 'completed'
    FAILED = 'failed'
    INTERRUPTED = 'interrupted'


ALLOWED_PARAMETERS = {
    'set_mode': {'mode', 'timeout'},
    'arm': {'timeout'},
    'takeoff': {'altitude_m', 'timeout', 'tolerance_m'},
    'land': {'timeout'},
}


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class MissionStep:
    """An explicit action and its inputs, prepared before mission execution."""

    action: str
    parameters: Mapping = field(default_factory=dict)

    def __post_init__(self):
        if self.action not in ALLOWED_PARAMETERS:
            raise ValueError(f'Unsupported mission action: {self.action}')
        parameters = dict(self.parameters)
        unknown = parameters.keys() - ALLOWED_PARAMETERS[self.action]
        if unknown:
            raise ValueError(f'Unexpected parameters for {self.action}: {sorted(unknown)}')
        if 'timeout' in parameters:
            timeout = parameters['timeout']
            if not finite_number(timeout) or timeout <= 0:
                raise ValueError('Step timeout must be finite and positive')
        if self.action == 'set_mode':
            mode = parameters.get('mode')
            if not isinstance(mode, str) or not mode.strip():
                raise ValueError('set_mode requires a nonempty mode string')
            parameters['mode'] = mode.strip().upper()
        if self.action == 'takeoff':
            altitude = parameters.get('altitude_m')
            tolerance = parameters.get('tolerance_m', 0.5)
            if not finite_number(altitude) or altitude <= 0:
                raise ValueError('takeoff requires a finite, positive altitude_m')
            if not finite_number(tolerance) or not 0 <= tolerance < altitude:
                raise ValueError('Takeoff tolerance must be nonnegative and less than altitude_m')
        object.__setattr__(self, 'parameters', MappingProxyType(parameters))
