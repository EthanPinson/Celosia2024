from commands2 import Command
from subsystems.drivesubsystem import DriveSubsystem

class FlipRewindTime(Command):
    def __init__(self, _drive: DriveSubsystem):
        super().__init__()
        self.drive = _drive

    def initialize(self):
        self.drive.isRewindTime = not self.drive.isRewindTime
