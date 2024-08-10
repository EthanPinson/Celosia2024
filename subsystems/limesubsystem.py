from commands2 import Subsystem
from networktables import NetworkTables
from wpimath.units import degreesToRadians as torad
from wpimath.geometry import Pose2d, Rotation2d
from typing import Sequence
from constants import LimeConstants as Lc
from requests import get
from websockets.sync.client import connect

# networktables is broken?????????

class LimeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._socket = connect(f'ws://{Lc.IP}:5806')
        self._limeNet = NetworkTables.getTable(Lc.NAME)

        self.botPose = None
        self.priTag: int = 0
        self.totalLatency: float = 0.0 # ms

    def periodic(self):
        response = get("http://" + Lc.IP + ":5807/results")
        if response.status_code != 200: self.botPose = None; return
        json = response.json()['Results']

        self.totalLatency = json['cl'] + json['tl']

        if len(fiducial := json['Fiducial']) == 0: self.priTag = Lc.NT_NUM_DEFAULT; self.botPose = None; return
        self.botPose = json['botpose']
        self.priTag = fiducial[0]['fID']
        self.offset = (fiducial[0]['tx'], fiducial[0]['ty'])

    @staticmethod
    def seqToPose(seq: Sequence[float]):
        # pulls x,y,yaw
        return None if seq is None else Pose2d(seq[0], seq[1], Rotation2d(torad(seq[5])))

    def calcTimestamp(self, tstampUS: int):
        # tstampUS is FPGA timestamp in microseconds
        tstampUS /= 1000
        return (tstampUS - self.totalLatency) if self.totalLatency > 0 else (tstampUS - Lc.DEFAULT_LATENCY)
    
    def disconnect(self): self._socket.close()