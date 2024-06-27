from commands2 import Subsystem, Command, cmd
from constants import IntakeConstants as IC
from rev import CANSparkMax

class IntakeSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    __isRunning: bool = False
    __isRunningRev: bool = False

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(IC.UPPER_ID)
        self.__lowerMotor = CANSparkMax(IC.LOWER_ID)

    def __setSpeed(self, mult: int):
        self.__lowerMotor.set(IC.ROLLER_DN_SPEED * mult)
        self.__upperMotor.set(IC.ROLLER_UP_SPEED * mult)

    def runIntake(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(1), self
        )
    
    def runIntakeRev(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(-1), self
        )
    
    def stopIntake(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(0), self
        )