from commands2 import Subsystem
from networktables import NetworkTables
from wpimath.units import degreesToRadians as torad
from wpimath.geometry import Pose2d, Rotation2d
from typing import Sequence
from constants import LimeConstants as Lc
from wpilib import RobotController

class LimeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.botPose = None
        self.totalLatency = 0 # ms
        self._limelightNet = NetworkTables.getTable(Lc.NAME)

    def periodic(self):
        self.botPose = self._limelightNet.getEntry("botpose").getDoubleArray(None)
        self.totalLatency = self._limelightNet.getNumber("cl") + self._limelightNet.getNumber("tl")

    @staticmethod
    def seqToPose(seq: Sequence[float]):
        # pulls x,y,yaw
        return None if seq is None else Pose2d(seq[0], seq[1], Rotation2d(torad(seq[5])))

    def calcTimestamp(self, tstampUS: int):
        # tstampUS is FPGA timestamp
        return (tstampUS / 1000) - self.totalLatency