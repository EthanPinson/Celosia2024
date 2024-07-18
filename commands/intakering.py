from commands2 import Command, WaitUntilCommand
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.feedersubsystem import FeederSubsystem
from subsystems.beamsubsystem import BeamSubsystem

class IntakeRing(Command):
    
    def __init__(self, intake: IntakeSubsystem, feeder: FeederSubsystem, beam: BeamSubsystem):
        super().__init__()
        self.feeder = feeder
        self.intake = intake
        self.beam = beam
        self.addRequirements(intake, feeder, beam)
    
    def initialize(self):
        self.feeder.setSpeed(0.25)
        self.intake.setSpeed(1)

    def isFinished(self):
        return self.beam.diginNot()
      
    def end(self, interrupted: bool):
        self.feeder.setSpeed(0)
        self.intake.setSpeed(0)
