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

    def setSpeed(self):
        self.__lowerMotor.set(FC.NOMINAL_SPEED)
        self.__upperMotor.set(FC.NOMINAL_SPEED)

    def stop(self):
        self.__lowerMotor.set(0.)
        self.__upperMotor.set(0.)

    def runFeeder(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.setSpeed()
        )
    
    def stopFeeder(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.stop()
        )