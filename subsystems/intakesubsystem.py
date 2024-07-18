from commands2 import Subsystem, Command, cmd
from constants import IntakeConstants as IC
from rev import CANSparkMax, CANSparkLowLevel

class IntakeSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    __isRunning: bool = False
    __isRunningRev: bool = False

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(IC.UPPER_ID, CANSparkLowLevel.MotorType.kBrushless)
        self.__lowerMotor = CANSparkMax(IC.LOWER_ID, CANSparkLowLevel.MotorType.kBrushless)

    def setSpeed(self, mult: int):
        self.__upperMotor.set(IC.ROLLER_UP_SPEED * mult)
        self.__lowerMotor.set(IC.ROLLER_DN_SPEED * mult)

    def setSpeeds(self, upper_speed: float, lower_speed: float):
        self.__upperMotor.set(upper_speed)
        self.__lowerMotor.set(lower_speed)

    def runIntake(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeeds(IC.ROLLER_UP_SPEED, IC.ROLLER_DN_SPEED), self
        )
    
    def runIntakeRev(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeeds(-1. * IC.ROLLER_UP_SPEED, -1. * IC.ROLLER_DN_SPEED), self
        )
    
    def stopIntake(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeeds(0., 0.), self
        )