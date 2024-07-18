#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.controller
import commands2
import commands2.cmd
import commands2.button

from commands.remmy import Remmy
from commands.shaggy import Shaggy
from commands.intakering import IntakeRing
from commands.getring import GetRing
from commands.shootring import ShootRing
from commands.ampring import AmpRing
import constants

#from subsystems.opticalsubsystem import OpticalSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.feedersubsystem import FeederSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.beamsubsystem import BeamSubsystem

from commands2.button import CommandXboxController

import commands.turntoangle
import commands.turntoangleprofiled
from commands.optics import Optics
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import *
from pathplannerlib.auto import PathPlannerAuto

from wpilib import CameraServer
from ntcore import NetworkTableInstance, NetworkTableEntry

from commands2 import SequentialCommandGroup, WaitCommand

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """
    drive: DriveSubsystem
    feeder: FeederSubsystem
    intake: IntakeSubsystem
    shooter: ShooterSubsystem
    beam: BeamSubsystem

    driverController: CommandXboxController
    opsController: CommandXboxController

    __camSelection: NetworkTableEntry

    # PathPlanner docs:
    #https://github.com/mjansen4857/pathplanner/wiki/Python-Example:-Build-an-Auto

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        self.drive = DriveSubsystem()
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()
        self.beam = BeamSubsystem()

        self.driverController = CommandXboxController(constants.OIConstants.kDriverControllerPort)
        self.opsController = CommandXboxController(constants.OIConstants.kOpsControllerPort)

        self.configureButtonBindings()

        self.__camSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection")
        CameraServer.launch("vision.py:main")

        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive.arcadeDrive(
                    -self.driverController.getLeftY(),
                    -self.driverController.getRightX(),
                ),
                self.drive,
            )
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """

        #self.driverController.b().onTrue(self.shooter.runShooter()) \
         #   .onFalse(self.shooter.stopShooter())
        
        #self.driverController.y().onTrue(self.intake.runIntake()) \
            #.onFalse(self.intake.stopIntake())

        self.driverController.rightBumper().whileTrue(IntakeRing(self.intake, self.feeder, self.beam))

        self.opsController.rightBumper().whileTrue(IntakeRing(self.intake, self.feeder, self.beam))

        self.opsController.y().whileTrue(ShootRing(self.feeder, self.shooter))

        self.opsController.b().whileTrue(AmpRing(self.feeder, self.shooter))

        self.opsController.a().whileTrue(self.feeder.runFeeder()) \
            .whileFalse(self.feeder.stopFeeder())
        
        self.opsController.x().whileTrue(self.feeder.runFeederRev()) \
            .whileFalse(self.feeder.stopFeeder())
        

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        #return commands2.InstantCommand()
        # return PathPlannerAuto('Example Auto')
        #return subsystems.drivesubsystem.sysIdDynamic(self.robotDrive, 1)
        #return Remmy(self.robotDrive)
        return Shaggy(self.robotDrive)