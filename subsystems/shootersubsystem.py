from commands2 import Subsystem
from constants import ShooterConstants as SC
from wpilib import Spark

class ShooterSubsystem(Subsystem):
    __innerMotor: Spark
    __outerMotor: Spark

    def __init__(self) -> None:
        super().__init__()

        self.__innerMotor = Spark(SC.INNER_ID)
        self.__outerMotor = Spark(SC.OUTER_ID)