import math
import numpy
import commands2

import constants

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.feedersubsystem import FeederSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

from commands2.button import CommandXboxController

from commands2 import Command, SequentialCommandGroup

class RobotContainer:
    driverController = CommandXboxController(0)
    opsController = CommandXboxController(1)

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        self.drive = DriveSubsystem()
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()

        self.configureBindings()

        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive.arcadeDrive(
                    -self.driverController.getLeftY(),
                    -(math.sqrt(abs(self.driverController.getRightX())) * numpy.sign(self.driverController.getRightX())),
                ),
                self.drive,
            )
        )

    def configureBindings(self) -> None:
        self.opsController.b() \
            .onTrue(self.intake.setSpeed(1.0)) \
            .onFalse(self.intake.setSpeed(0.0))
        
        self.opsController.a() \
            .onTrue(self.feeder.setSpeed(1.0)) \
            .onFalse(self.feeder.setSpeed(0.0))
        
        self.opsController.x() \
            .onTrue(self.shooter.setSpeed(1.0)) \
            .onFalse(self.shooter.setSpeed(0.0))
        
        self.opsController.y() \
            .onTrue(SequentialCommandGroup(
                self.intake.setSpeed(-0.5),
                self.feeder.setSpeed(-0.5),
                self.shooter.setSpeed(-0.5))) \
            .onFalse(SequentialCommandGroup(
                self.intake.setSpeed(0.0),
                self.feeder.setSpeed(0.0),
                self.shooter.setSpeed(0.0)))

    def getAutonomousCommand(self) -> Command:
        pass