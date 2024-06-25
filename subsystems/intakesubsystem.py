from commands2 import Subsystem, Command
import commands2.cmd
from constants import IntakeConstants as IC
from rev import CANSparkMax

class IntakeSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(IC.UPPER_ID)
        self.__lowerMotor = CANSparkMax(IC.LOWER_ID)

    def setSpeed(self):
        self.__lowerMotor.set(IC.ROLLER_DN_SPEED)
        self.__upperMotor.set(IC.ROLLER_UP_SPEED)

    def stop(self):
        self.__lowerMotor.set(0.)
        self.__upperMotor.set(0.)

    def runIntake(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.setSpeed()
        )
    
    def stopIntake(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.stop()
        )