from commands2 import SequentialCommandGroup, WaitUntilCommand
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.feedersubsystem import FeederSubsystem
from subsystems.beamsubsystem import BeamSubsystem

class GetRing(SequentialCommandGroup):
    def __init__(self, intake: IntakeSubsystem, feeder: FeederSubsystem, beam: BeamSubsystem):
        super().__init__(
            intake.runIntake(),
            feeder.runFeeder(),
            WaitUntilCommand(beam.diginNot),
            feeder.stopFeeder(),
            intake.stopIntake())