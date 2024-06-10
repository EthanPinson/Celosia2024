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

from RamseteCommand import RamseteCommand
from commands.remmy import Remmy
import constants
import subsystems.drivesubsystem
from subsystems.opticalsubsystem import OpticalSubsystem
import commands.turntoangle
import commands.turntoangleprofiled
from commands.optics import Optics
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import *
from pathplannerlib.auto import PathPlannerAuto

from commands2 import Command
from RamseteCommand import RamseteCommand
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import commands2

from wpimath.controller import RamseteController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint

from constants import AutoConstants
from wpimath.controller import PIDController


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.

    """

    # PathPlanner docs:
    #https://github.com/mjansen4857/pathplanner/wiki/Python-Example:-Build-an-Auto

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        # The robot's subsystems
        self.robotDrive = subsystems.drivesubsystem.DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(
            constants.OIConstants.kDriverControllerPort
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.RunCommand(
                lambda: self.robotDrive.arcadeDrive(
                    -self.driverController.getLeftY(),
                    -self.driverController.getRightX(),
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        # Drive at half speed when the right bumper is held
        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kRightBumper
        ).onTrue(
            commands2.InstantCommand(
                (lambda: self.robotDrive.setMaxOutput(0.25)), self.robotDrive
            )
        ).onFalse(
            commands2.InstantCommand(
                (lambda: self.robotDrive.setMaxOutput(0.5)), self.robotDrive
            )
        )

        # Stabilize robot to drive straight with gyro when left bumper is held
        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kLeftBumper
        ).whileTrue(
            commands2.PIDCommand(
                wpimath.controller.PIDController(
                    constants.DriveConstants.kStabilizationP,
                    constants.DriveConstants.kStabilizationI,
                    constants.DriveConstants.kStabilizationD,
                ),
                # Close the loop on the turn rate
                self.robotDrive.getTurnRate,
                # Setpoint is 0
                0,
                # Pipe the output to the turning controls
                lambda output: self.robotDrive.arcadeDrive(
                    -self.driverController.getLeftY(), output
                ),
                # Require the robot drive
                self.robotDrive,
            )
        )

        # Turn to 90 degrees__init__when the 'X' button is pressed, with a 5 second timeout

        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kA
        ).onTrue(commands.turntoangle.TurnToAngle(math.pi/2, self.robotDrive).withTimeout(5))

        # Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kB
        ).onTrue(
            commands.turntoangleprofiled.TurnToAngleProfiled(
                -90, self.robotDrive
            ).withTimeout(5)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        #return commands2.InstantCommand()
        # return PathPlannerAuto('Example Auto')
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            start=Pose2d(0, 0, Rotation2d(0)),
            interiorWaypoints=[Translation2d(1, 1), Translation2d(2, -1)],
            end=Pose2d(3, 0, Rotation2d(0)),
            config=self.trajectoryConfig
        )
        return RamseteCommand(
            exampleTrajectory, self.getPose2d, self.ramsete, self.feedforward,
            AutoConstants.kDriveKinematics, self.getCurrentSpeeds,
            PIDController(AutoConstants.kPDriveVel, 0, 0),
            PIDController(AutoConstants.kPDriveVel, 0, 0),
            10, self)






