import math
import numpy
import commands2

import constants

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.feedersubsystem import FeederSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.opticalsubsystem import OpticalSubsystem
from subsystems.limesubsystem import LimeSubsystem

from commands2.button import CommandXboxController

from commands2 import Command, SequentialCommandGroup, WaitCommand, ParallelCommandGroup

class RobotContainer:
    driverController = CommandXboxController(0)
    opsController = CommandXboxController(1)

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        self.optics = OpticalSubsystem()
        self.lime = LimeSubsystem()
        self.drive = DriveSubsystem(self.optics, self.lime)
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()

        self.configureBindings()

        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive.arcade(
                    -self.driverController.getLeftY(),
                    -self.driverController.getRightX(),
                ),
                self.drive,
            )
        )

    def configureBindings(self) -> None:
        self.opsController.b() \
            .onTrue(self.intake.setSpeed(0.5)) \
            .onFalse(self.intake.setSpeed(0.0))
        
        self.opsController.a() \
            .onTrue(self.feeder.setSpeed(1.0)) \
            .onFalse(self.feeder.setSpeed(0.0))
        
        self.opsController.x() \
            .onTrue(self.shooter.setSpeed(1.0)) \
            .onFalse(self.shooter.setSpeed(0.0))
        
        self.opsController.y() \
            .onTrue(ParallelCommandGroup(
                self.intake.setSpeed(-0.5),
                self.feeder.setSpeed(-0.5),
                self.shooter.setSpeed(-0.5))) \
            .onFalse(ParallelCommandGroup(
                self.intake.setSpeed(0.0),
                self.feeder.setSpeed(0.0),
                self.shooter.setSpeed(0.0)))

        self.opsController.rightBumper() \
            .onTrue(SequentialCommandGroup(
                self.intake.setSpeed(-0.5),
                WaitCommand(0.5),
                self.intake.setSpeed(0.0)))
        
        self.driverController.leftBumper().onTrue(self.drive.toggleRewindTime())

    def getAutonomousCommand(self) -> Command:
        pass

    #def cleanup(self): self.lime.disconnect()