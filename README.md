# Drone Autonomy Lab

Learning to build autonomous drones through ArduPilot, Python, simulation,
hardware, and vision. Indoor flight is the first hardware goal; outdoor flight
may follow later.

## Progress

| Capability | Status |
| --- | --- |
| MAVProxy takeoff, hover, movement, landing | Completed in SITL by the author |
| Python GUIDED, arming, takeoff to 10 m, landing/disarming | Completed in SITL by the author |
| Continuous telemetry monitor alongside the mission | Completed in SITL by the author |
| Mission executor: validated steps, progress, failure/interruption tracking | Implemented and mock-tested |
| Package structure and command-line entry points | Current checkpoint; validated by installation and tests |
| Coordinate planning, waypoint arrival, square mission | Next |
| Automatic mission recovery and hardware flight | Future work |

Prior SITL exercises were reported successful by the author; their flight logs
are not included. This checkpoint's verification uses mocked vehicle connections
and synthetic telemetry. The refactored flight cycle still needs a live SITL
rerun. This project has not been validated on a physical aircraft.

## Project structure

```text
drone-autonomy-lab/
├── src/drone_autonomy_lab/
│   ├── __init__.py
│   ├── checks.py                 # ACK and observed-state checks
│   ├── vehicle.py                # ArduPilot connection and vehicle operations
│   ├── missions/
│   │   ├── __init__.py
│   │   ├── models.py             # MissionStep validation and MissionState
│   │   ├── executor.py           # Ordered execution and progress tracking
│   │   └── demo.py               # Explicit takeoff/landing plan and CLI
│   └── telemetry/
│       ├── __init__.py
│       ├── state.py              # Latest values, units, and reception ages
│       └── monitor.py            # Receive loop and terminal display
├── tests/                        # Mocked vehicle and telemetry tests
├── pyproject.toml                # Package metadata, dependencies, CLI commands
├── requirements.txt              # Convenience editable install
├── requirements-dev.txt          # Convenience editable install with dev tools
├── .flake8
├── .gitignore
└── README.md
```

Mission planning will convert human instructions into fixed coordinate targets
and optional heading. The mission executor sequences explicit actions and records
their completion. The vehicle interface encodes and sends MAVLink inputs.
Telemetry independently observes vehicle state.

The current vehicle methods still combine sending commands with waiting for
telemetry confirmation. Separating those waits from the receiver is future work;
this package organization establishes the boundaries for that change. There is
no coordinate planner yet. A full architecture/sequence diagram is deferred until
those components are implemented.

## Setup

Use Linux, Python 3.12 or newer, and a separate ArduPilot checkout with working
Copter SITL. Follow the [official SITL setup guide](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
for ArduPilot prerequisites.

The existing exercises used Ubuntu 24.04, Python 3.12, pymavlink 2.4.50, and
ArduPilot revision `c5d75e9e77db186af9efd49c66ce8cda520dd732`. Other firmware
revisions have not been validated here.

```bash
git clone https://github.com/shiv4m25/drone-autonomy-lab.git
cd drone-autonomy-lab
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
```

Dependencies are defined in `pyproject.toml`. The editable install makes source
changes available without reinstalling. The `dev` extra includes flake8; for only
runtime dependencies, use `python -m pip install -e .`.

In VS Code, select `.venv/bin/python` as the project's Python interpreter.

## Run the SITL mission

In another terminal, use ArduPilot's own configured environment and start SITL:

```bash
cd /path/to/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

In the MAVProxy prompt, add the mission and monitor output ports once per session:

```text
output add 127.0.0.1:14560
output add 127.0.0.1:14561
```

In the project terminal with `.venv` activated:

```bash
drone-mission
```

The mission runs **GUIDED → arm → take off to 10 m → LAND → wait for disarming**.
It requests landing immediately after the takeoff altitude threshold is reached.
The executor prints each step and reports completion or the failed/interrupted
step. All localhost connections assume MAVProxy runs on the same computer.

## Run the telemetry monitor

In a separate project terminal:

```bash
source .venv/bin/activate
drone-telemetry
```

It receives on `127.0.0.1:14561` independently of the mission on `14560`. It sends
no vehicle commands. Press Ctrl+C to stop.

The display includes mode/armed state, position/velocity, attitude, battery,
sensor health, GPS, and the last status text. Each value has a reception age;
values older than five seconds are marked `STALE`. Missing messages are marked
`not received`; measurements reported unavailable are shown as `unknown`.

```bash
drone-telemetry --refresh 0.5 --stale-after 5
drone-telemetry --duration 10
drone-telemetry --help
```

Use `--connection` to override the monitor's default
`udpin:127.0.0.1:14561`. Stream rates come from MAVProxy/ArduPilot; the monitor
uses the available streams without requesting new rates. See
[ArduPilot's stream-rate documentation](https://ardupilot.org/dev/docs/mavlink-requesting-data.html)
if a required message is absent.

## Mission model and executor

The current demo creates a plan before connecting:

```python
from drone_autonomy_lab.missions.demo import takeoff_land_mission
from drone_autonomy_lab.missions.executor import MissionExecutor

mission = takeoff_land_mission(altitude_m=10)
MissionExecutor(vehicle).run(mission)
```

Steps contain an action and immutable parameter mapping. Supported actions are
`set_mode`, `arm`, `takeoff`, and `land`. Input validation happens when each step
is created. The executor checks that actions exist before running the plan.

Its states are `READY`, `RUNNING`, `COMPLETED`, `FAILED`, and `INTERRUPTED`.
`current_step`, `completed_steps`, and `error` provide execution progress.
Vehicle methods return after confirmation or raise on error/timeout. Exceptions
stop subsequent steps and retain the original cause. Use a new executor for a
new mission; there is no automatic retry. The caller owns connection cleanup.

## Tests

With the development environment activated:

```bash
python -m unittest discover -s tests -v
flake8 src tests
```

The current suite contains 27 tests and sends no vehicle commands. It covers
unit conversions, unknown/stale telemetry, source filtering, acknowledgments,
timeouts, takeoff/landing confirmation, executor ordering, interruption, and
cleanup. Lint settings live in `.flake8`.

Module execution is also supported after installation:

```bash
python -m drone_autonomy_lab.missions.demo --help
python -m drone_autonomy_lab.telemetry.monitor --help
```

## Behavior and limitations

- The default takeoff completes at or above 9.5 m relative to home, with one
  30-second deadline for ACK and altitude confirmation. This is a climb threshold,
  not proof of a settled hover.
- Landing waits up to 90 seconds for LAND mode and a disarmed heartbeat. It does
  not send a forced-disarm command.
- Local coordinates use the estimator origin; altitude relative to home is not
  a direct measurement above terrain. N/E/D means north/east/down.
- Closing the connection does not land the vehicle. If a step fails or is
  interrupted, later steps do not run. Use `mode land` in MAVProxy and wait for
  disarming. Automatic recovery and command retries remain future work.
- Reception age is measured locally; it does not establish network latency or
  whether queued data represents the vehicle's current state.
- The monitor shows the latest battery ID and status-text chunk. It does not
  reassemble long status messages; the last warning is historical, not a current
  fault diagnosis. Sensor-health masks, system status, and GPS fix type are enums
  or bitmasks defined in the [MAVLink reference](https://mavlink.io/en/messages/common.html).

## Next milestones

1. Separate mission confirmation from the vehicle's MAVLink receive loop.
2. Implement coordinate targets, arrival checks, and a complete square mission.
3. Test link/position loss and define explicit mission recovery behavior.
4. Develop piloting skills and select compatible hardware.
5. Establish indoor positioning before physical autonomy, then investigate vision.

## AI assistance

The author develops this project with OpenAI Codex assistance for explanations,
implementation, debugging, documentation, and tests. The author runs the SITL
exercises and remains responsible for understanding and reviewing the code.
