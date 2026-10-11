# AP_FLAKE8_CLEAN
"""Display MAVLink telemetry without sending vehicle commands."""
import argparse
import math
import sys
import time

from pymavlink import mavutil

from .receiver import TelemetryReceiver


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def monitor(connection, refresh=1, stale_after=5, duration=None):
    receiver = TelemetryReceiver(connection).start()
    started = time.monotonic()
    stop_at = started + duration if duration is not None else math.inf
    try:
        while True:
            now = time.monotonic()
            if now >= stop_at:
                return
            prefix = '\033[2J\033[H' if sys.stdout.isatty() else ''
            print(prefix + receiver.render(now, stale_after), flush=True)
            time.sleep(min(refresh, stop_at - now))
    finally:
        receiver.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='udpin:127.0.0.1:14561')
    parser.add_argument('--refresh', type=positive_number, default=1, help='display interval in seconds (default: 1)')
    parser.add_argument('--stale-after', type=positive_number, default=5, help='stale threshold in seconds (default: 5)')
    parser.add_argument('--duration', type=positive_number, help='optional run time in seconds')
    args = parser.parse_args()
    connection = mavutil.mavlink_connection(args.connection)
    try:
        monitor(connection, args.refresh, args.stale_after, args.duration)
    except KeyboardInterrupt:
        print('\nTelemetry monitor stopped.')
    finally:
        connection.close()


if __name__ == '__main__':
    main()
