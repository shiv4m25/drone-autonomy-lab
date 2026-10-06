# Drone Autonomy Lab

Learning to build autonomous drones through ArduPilot, Python, simulation,
hardware, and vision. The project starts indoors, with outdoor flight a possible
later phase.

## Current milestone

A Python script runs an ArduCopter SITL flight:

**Connect → GUIDED → arm → take off to 10 m → LAND → confirm disarmed.**

The author has completed this cycle in SITL. Automated tests use mocked
connections; flight logs for the manual validation are not included.

This is a learning project currently developed for SITL. It has not been
validated on a physical aircraft.

## How it works

ArduPilot runs the flight control and simulated physics. MAVProxy provides the
terminal/map and forwards MAVLink telemetry. The Python script sends commands
and checks telemetry to confirm their results.

```text
ArduCopter SITL ↔ MAVProxy ↔ Python / pymavlink
                            UDP 127.0.0.1:14560
```

- `drone_connect.py`: connection, mode selection, arming, takeoff, and landing.
- `command_utils.py`: acknowledgment and vehicle-state checks.
- `test_takeoff.py`, `test_land.py`: mocked tests for thresholds, timeouts,
  rejected commands, telemetry filtering, sequence, and cleanup.

## Setup

Use Linux with a separate ArduPilot checkout and a working Copter SITL setup.
Follow the [official SITL setup guide](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
to install its prerequisites. ArduPilot source is not bundled here.

The development environment used Python 3.12, pymavlink 2.4.50, and ArduPilot
revision `c5d75e9e77db186af9efd49c66ce8cda520dd732`. This revision is recorded for
reproduction; compatibility with other revisions has not been established.

Clone this project, then create its Python environment:

```bash
git clone https://github.com/shiv4m25/drone-autonomy-lab.git
cd drone-autonomy-lab
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Start SITL in another terminal from the ArduPilot checkout, using its own
configured Python environment:

```bash
cd /path/to/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

In the MAVProxy prompt, forward messages to this script:

```text
output add 127.0.0.1:14560
```

Keep SITL running. In this project's terminal, run:

```bash
python drone_connect.py
```

This command arms the simulated vehicle, climbs to 10 m relative to home,
then immediately requests landing once the takeoff threshold is reached.

## Tests

```bash
python -m pip install -r requirements-dev.txt
python -m unittest discover -s . -p 'test_*.py' -v
flake8 --max-line-length=127 .
```

The 11 current tests run without SITL and do not send vehicle commands.

## Behavior and limitations

- Takeoff waits for an accepted acknowledgment and altitude of at least 9.5 m
  for the default 10 m target. ACK and climb share a 30-second deadline.
- Landing waits up to 90 seconds for LAND mode and a disarmed heartbeat from
  the selected flight-controller component.
- Altitude telemetry is relative to home, not a direct measurement above terrain.
- Closing a MAVLink connection does not land the vehicle. If an earlier step
  raises an exception, the script closes the connection without automatic
  recovery. Use `mode land` in MAVProxy and wait for disarming.
- Command retries, robust stale-message handling, loss-of-position/link recovery,
  waypoints, and physical flight readiness are still future work.

## Next steps

1. Fly to a destination and detect arrival, then fly a square in SITL.
2. Test failures and implement explicit mission recovery behavior.
3. Develop piloting skills and choose compatible hardware.
4. Establish reliable indoor positioning before physical autonomous flight.
5. Explore vision and onboard compute, with outdoor flight a possible later phase.

## AI assistance

This project is being developed by its author with OpenAI Codex assistance for
explanations, implementation, debugging, documentation, and tests. The author
runs the simulation exercises and remains responsible for understanding and
reviewing the code.
