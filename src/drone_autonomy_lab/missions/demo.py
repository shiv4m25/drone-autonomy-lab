# AP_FLAKE8_CLEAN
"""Run the current takeoff-and-landing demo against Copter SITL."""
import argparse

from drone_autonomy_lab.missions.executor import MissionExecutor
from drone_autonomy_lab.missions.models import MissionStep
from drone_autonomy_lab.vehicle import Drone


def takeoff_land_mission(altitude_m=10):
    return (
        MissionStep("set_mode", {"mode": "GUIDED"}),
        MissionStep("arm"),
        MissionStep("takeoff", {"altitude_m": altitude_m}),
        MissionStep("land"),
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)
    mission = takeoff_land_mission()
    drone = Drone()
    try:
        MissionExecutor(drone).run(mission)
    finally:
        drone.close()


if __name__ == "__main__":
    main()
