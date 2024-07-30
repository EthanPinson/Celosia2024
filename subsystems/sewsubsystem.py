from wpimath.trajectory import TrajectoryUtil, Trajectory
from wpilib import getDeployDirectory
from commands2 import Subsystem
from os.path import join

class SewSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._deployPath = getDeployDirectory()

    def readJson(self, name) -> Trajectory:
        jsonpath = join(self._deployPath, "/paths", name + ".wpilib.json")
        return TrajectoryUtil.fromPathweaverJson(jsonpath)