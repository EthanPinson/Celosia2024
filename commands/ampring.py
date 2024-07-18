from commands2 import SequentialCommandGroup, WaitCommand, WaitUntilCommand, Command
from wpilib import Timer

from subsystems.feedersubsystem import FeederSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.beamsubsystem import BeamSubsystem
from constants import FeederConstants as FC
from constants import ShooterConstants as SC

class AmpRing(Command):
    def __init__(self, feeder: FeederSubsystem, shooter: ShooterSubsystem):
        super().__init__()
        self.feeder = feeder
        self.shooter = shooter
        self.addRequirements(feeder, shooter)
        self.timer = Timer()

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.shooter.setSpeed(SC.AMP_SPEED)
        if self.timer.get() > 0.5:
            self.feeder.setSpeed(FC.AMP_SPEED)
            
    def end(self, interrupted):
        self.shooter.setSpeed(0.0)
        self.feeder.setSpeed(0.0)