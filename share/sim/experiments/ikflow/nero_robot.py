"""Register the NERO arm (kinematics only, flange = link7) with jrl so IKFlow can use it."""
from pathlib import Path

import jrl.robots as robots
from jrl.robot import Robot

URDF = str(Path(__file__).resolve().parent / "nero_kin.urdf")


class Nero(Robot):
    name = "nero"
    formal_robot_name = "NERO"
    POSITIONAL_REPEATABILITY_MM = 0.1
    ROTATIONAL_REPEATABILITY_DEG = 0.1

    def __init__(self, verbose=False):
        links = ["base_link"] + [f"link{i}" for i in range(1, 8)]
        Robot.__init__(self, Nero.name, URDF, [f"joint{i}" for i in range(1, 8)], "base_link", "link7",
                       [], {l: None for l in links}, verbose=verbose)


if Nero not in robots.ALL_CLCS:
    robots.ALL_CLCS.append(Nero)
    robots.ALL_ROBOT_NAMES.append(Nero.name)
