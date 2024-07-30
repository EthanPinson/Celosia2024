from wpimath.trajectory import TrajectoryUtil, Trajectory
from wpilib import getOperatingDirectory
from commands2 import Subsystem
from os.path import join

class SewSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._opPath = getOperatingDirectory()

    def readJson(self, name) -> Trajectory:
        jsonpath = join(self._opPath, "/PathWeaver/output", name + ".wpilib.json")
        return TrajectoryUtil.fromPathweaverJson(jsonpath)