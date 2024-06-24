from commands2 import Subsystem
from constants import FeederConstants as FC
from rev import CANSparkMax

class FeederSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(FC.UPPER_ID)
        self.__lowerMotor = CANSparkMax(FC.LOWER_ID)