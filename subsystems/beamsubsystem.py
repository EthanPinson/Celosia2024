from commands2 import Subsystem
from wpilib import DigitalInput
from constants import BeamConstants

class BeamSubsystem(Subsystem):
    digin: DigitalInput

    def __init__(self):
        super().__init__()

        self.digin = DigitalInput(BeamConstants.CHANNEL)

    def diginNot(self) -> bool:
        return not self.digin.get()