from commands2 import SequentialCommandGroup, WaitCommand, WaitUntilCommand, Command
from subsystems.feedersubsystem import FeederSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.beamsubsystem import BeamSubsystem
from wpilib import Timer

class ShootRing(SequentialCommandGroup):
    def __init__(self, feeder: FeederSubsystem, shooter: ShooterSubsystem, beam: BeamSubsystem):
        super().__init__(
            shooter.runShooter(),
            WaitCommand(1),
            feeder.runFeeder(),
            WaitUntilCommand(beam.digin.get),
            WaitCommand(1),
            feeder.stopFeeder(),
            shooter.stopShooter())
        
class ShootRing2(Command):
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
        self.shooter.setSpeed(1)
        if self.timer.get() > 0.5:
            self.feeder.setSpeed(1.0)
            
    def end(self, interrupted):
        self.shooter.setSpeed(0)
        self.feeder.setSpeed(0.0)
