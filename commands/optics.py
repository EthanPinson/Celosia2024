from commands2 import Command
from subsystems.opticalsubsystem import OpticalSubsystem

class Optics(Command):
    def __init__(self, opticalsubsystem: OpticalSubsystem):
        self.eyeballs = opticalsubsystem()
        return

    def isFinished(self) -> bool:
        return False
    
    def checkTarget(self):
        targets = self.eyeballs.blueTargets
        # print(targets)