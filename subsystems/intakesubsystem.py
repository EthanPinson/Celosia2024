from commands2 import Subsystem, Command, cmd
from constants import IntakeConstants as IC
from rev import CANSparkMax, CANSparkLowLevel

class IntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.__motorA = CANSparkMax(IC.UPPER_ID, CANSparkLowLevel.MotorType.kBrushless)
        self.__motorB = CANSparkMax(IC.LOWER_ID, CANSparkLowLevel.MotorType.kBrushless)
        self.__motorB.follow(self.__motorA)

    def setSpeed(self, speed: float) -> Command:
        return cmd.runOnce(lambda: self.__motorA.set(speed), self)