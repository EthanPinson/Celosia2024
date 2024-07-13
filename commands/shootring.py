from commands2 import SequentialCommandGroup, WaitCommand, WaitUntilCommand, Command
from subsystems.feedersubsystem import FeederSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.beamsubsystem import BeamSubsystem

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
    def __init__(self, feeder, shooter, beam):
        super().__init__()