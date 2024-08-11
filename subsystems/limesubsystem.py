from commands2 import Subsystem
from networktables import NetworkTables
from wpimath.units import degreesToRadians as torad
from wpimath.geometry import Pose2d, Rotation2d
from typing import Sequence
from constants import LimeConstants as Lc
from pymodbus.client import ModbusTcpClient
from wpilib import DataLogManager
from struct import pack, unpack

# networktables is broken?????????

class LimeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._limeNet = NetworkTables.getTable(Lc.NAME)

        self.botPose = None
        self.priTag: int = 0
        self.totalLatency: float = 0.0 # ms

        self._fails: int = 0

    def __aux_init(self):
        self._tcpClient = ModbusTcpClient(f'{Lc.IP}:502')
        self._tcpClient.connect()
        self._fails = -1

    def periodic(self):
        if self._fails >= Lc.AUX_THRESHOLD: self.__aux_init(); DataLogManager.log("[LimeSubsystem]: SWITCHING TO AUX PERIODIC")
        if self._fails < 0: self.__aux_periodic(); return
        try: self.__pri_periodic(); self._fails = 0
        except: self._fails += 1

    def __pri_periodic(self):
        self.botPose = self._limeNet.getNumberArray("botpose", None)
        self.priTag = self._limeNet.getNumber("tid", -1)
        self.totalLatency = self._limeNet.getNumber("tl", -1), self._limeNet.getNumber("cl", -1)

        if self.totalLatency > 0: raise Exception()

    def __aux_periodic(self):
        regs = self._tcpClient.read_input_registers(0x00, 26).registers
        
        self.priTag = regs[26]
        self.totalLatency = regs[10] + regs[11]

        tx = unpack('f', pack('I', (regs[17] << 16) | regs[16]))[0]
        ty = unpack('f', pack('I', (regs[19] << 16) | regs[18]))[0]

    @staticmethod
    def seqToPose(seq: Sequence[float]):
        # pulls x,y,yaw
        return None if seq is None else Pose2d(seq[0], seq[1], Rotation2d(torad(seq[5])))

    def calcTimestamp(self, tstampUS: int):
        # tstampUS is FPGA timestamp in microseconds
        tstampUS /= 1000
        return (tstampUS - self.totalLatency) if self.totalLatency > 0 else (tstampUS - Lc.DEFAULT_LATENCY)
    
    def disconnect(self): pass