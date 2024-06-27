from commands2 import Subsystem, RunCommand, Command
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

    def __setSpeedx(self, mult: int):
        self.__lowerMotor.set(IC.ROLLER_DN_SPEED * mult)
        self.__upperMotor.set(IC.ROLLER_UP_SPEED * mult)

    def runIntake(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.__setSpeedx(1)
        )
    
    def stopIntake(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.__setSpeedx(0)
        )