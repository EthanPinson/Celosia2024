from commands2 import Subsystem
from networktables import NetworkTables
from wpimath.units import degreesToRadians as torad
from wpimath.geometry import Pose2d, Rotation2d
from typing import Sequence
from constants import LimeConstants as Lc

class LimeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._limeNet = NetworkTables.getTable(Lc.NAME)

        self.botPose = None
        self.priTag = 0
        self.totalLatency = 0 # ms

    def periodic(self):
        self.botPose = self._limeNet.getNumberArray("botpose", None)
        self.priTag = self._limeNet.getNumber("tid", Lc.NT_NUM_DEFAULT)
        self.totalLatency = self._limeNet.getNumber("cl", Lc.NT_NUM_DEFAULT) + self._limeNet.getNumber("tl", Lc.NT_NUM_DEFAULT)

    @staticmethod
    def seqToPose(seq: Sequence[float]):
        # pulls x,y,yaw
        return None if seq is None else Pose2d(seq[0], seq[1], Rotation2d(torad(seq[5])))

    def calcTimestamp(self, tstampUS: int):
        # tstampUS is FPGA timestamp in microseconds
        tstampUS /= 1000
        return (tstampUS - self.totalLatency) if self.totalLatency > 0 else (tstampUS - Lc.DEFAULT_LATENCY)