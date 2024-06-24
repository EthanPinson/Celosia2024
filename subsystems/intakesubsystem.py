from commands2 import Subsystem
from constants import IntakeConstants as IC
from rev import CANSparkMax

class IntakeSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(IC.UPPER_ID)
        self.__lowerMotor = CANSparkMax(IC.LOWER_ID)