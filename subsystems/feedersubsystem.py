from commands2 import Subsystem, Command
import commands2.cmd
from constants import FeederConstants as FC
from rev import CANSparkMax

class FeederSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(FC.UPPER_ID)
        self.__lowerMotor = CANSparkMax(FC.LOWER_ID)

    def __setSpeed(self, mult: int):
        self.__lowerMotor.set(FC.NOMINAL_SPEED * mult)
        self.__upperMotor.set(FC.NOMINAL_SPEED * mult)

    def runFeeder(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.__setSpeed(1)
        )
    
    def stopFeeder(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.__setSpeed(0)
        )