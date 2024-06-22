from commands2 import Subsystem
from constants import FeederConstants as FC
#from wpilib import can

class FeederSubsystem(Subsystem):
    __upperMotor: None
    __lowerMotor: None

    def __init__(self) -> None:
        super().__init__()

        #self.__innerMotor = Spark(SC.INNER_ID)
        #self.__outerMotor = Spark(SC.OUTER_ID)