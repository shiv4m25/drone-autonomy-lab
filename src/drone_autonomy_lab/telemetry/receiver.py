# AP_FLAKE8_CLEAN
"""One continuous MAVLink reader with bounded observations and blocking waits."""
import math
import threading
import time
from collections import deque
from dataclasses import dataclass

from .state import MESSAGE_TYPES, TelemetryState


@dataclass(frozen=True)
class Observation:
    sequence: int
    received_at: float
    message: object


class TelemetryReceiver:
    """Own receives; never send commands or close the caller's connection."""

    def __init__(self, connection):
        self.connection = connection
        self._state = TelemetryState()
        self._events = deque(maxlen=2048)
        self._sequence = 0
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread = None
        self._error = None

    def start(self):
        if self._thread is not None or self._stop.is_set():
            raise RuntimeError('Use a new receiver for each connection session')
        self._thread = threading.Thread(target=self._receive, name='mavlink-telemetry', daemon=True)
        self._thread.start()
        return self

    def _receive(self):
        try:
            while not self._stop.is_set():
                message = self.connection.recv_match(blocking=True, timeout=0.2)
                if message is not None:
                    self._record(message, time.monotonic())
        except Exception as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()

    def _record(self, message, received_at):
        with self._condition:
            self._state.update(message, received_at)
            source = (message.get_srcSystem(), message.get_srcComponent())
            if source != self._state.source:
                return
            if message.get_type() not in MESSAGE_TYPES | {'COMMAND_ACK'}:
                return
            self._sequence += 1
            self._events.append(Observation(self._sequence, received_at, message))
            self._condition.notify_all()

    @property
    def source(self):
        with self._condition:
            return self._state.source

    def _check_available(self):
        if self._error is not None:
            raise RuntimeError('Telemetry receiver failed') from self._error
        if self._stop.is_set():
            raise RuntimeError('Telemetry receiver stopped')

    def mark(self):
        """Exclude observations already recorded before an action is sent."""
        with self._condition:
            self._check_available()
            return self._sequence

    def wait_for(self, message_type, check, *, after, deadline, description, max_age=5):
        """Wait on received observations, without reading the MAVLink connection."""
        if not math.isfinite(deadline) or not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Deadline must be finite and max_age finite and positive')
        cursor = after
        with self._condition:
            while True:
                self._check_available()
                now = time.monotonic()
                if now >= deadline:
                    raise TimeoutError(f'Timed out waiting for {description}')
                if self._events and cursor < self._events[0].sequence - 1:
                    raise RuntimeError('Telemetry observation history overflowed during confirmation')
                for event in self._events:
                    if event.sequence <= cursor:
                        continue
                    cursor = event.sequence
                    if now - event.received_at >= max_age:
                        continue
                    if event.message.get_type() == message_type and check(event.message):
                        return event.message
                self._condition.wait(timeout=min(0.2, deadline - now))

    def render(self, now, stale_after):
        with self._condition:
            self._check_available()
            return self._state.render(now, stale_after)

    def close(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1)
            if self._thread.is_alive():
                raise RuntimeError('Telemetry receiver did not stop within one second')
