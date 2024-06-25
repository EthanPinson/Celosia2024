from commands2 import Subsystem, RunCommand
from constants import IntakeConstants as IC
from rev import CANSparkMax

class IntakeSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(IC.UPPER_ID)
        self.__lowerMotor = CANSparkMax(IC.LOWER_ID)

    def setIn(self) -> RunCommand:
        return RunCommand(lambda: print("works"))
    
    def setOut(self) -> RunCommand:
        return RunCommand(lambda: print("also works"))