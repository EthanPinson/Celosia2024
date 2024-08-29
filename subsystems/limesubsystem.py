from commands2 import Subsystem
from networktables import NetworkTables
from wpimath.units import degreesToRadians as torad
from wpimath.geometry import Pose2d, Rotation2d
from typing import Sequence
from constants import LimeConstants as Lc
from wpilib import DataLogManager

class LimeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._limeNet = NetworkTables.getTable(Lc.NAME)

        self.botPose = None
        self.priTag: int = 0
        self.totalLatency: float = 0.0 # ms

    def periodic(self):
        self.botPose = self._limeNet.getNumberArray("botpose", None)
        self.priTag = self._limeNet.getNumber("tid", -1)
        self.totalLatency = self._limeNet.getValue("tl", -1) + self._limeNet.getValue("cl", -1)
        print(self.totalLatency)

    @staticmethod
    def seqToPose(seq: Sequence[float]):
        # pulls x,y,yaw
        return None if seq is None else Pose2d(seq[0], seq[1], Rotation2d(torad(seq[5])))

    def calcTimestamp(self, tstampUS: int):
        # tstampUS is FPGA timestamp in microseconds
        tstampUS /= 1000
        return (tstampUS - self.totalLatency) if self.totalLatency > 0 else (tstampUS - Lc.DEFAULT_LATENCY)
    
    def disconnect(self): pass