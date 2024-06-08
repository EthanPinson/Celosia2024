#
# Copyright (c) FIRSTfrom pathplannerlib.auto import AutoBuilder
import pathplannerlib
from pathplannerlib.config import *
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.config import *
from pathplannerlib.auto import AutoBuilder
from wpilib import DriverStation
import wpilib.drive
import wpimath.estimator
import commands2
import math
import navx
import wpimath
import wpimath.geometry
from  wpilib.shuffleboard import *
import constants
import wpimath.kinematics
from .opticalsubsystem import OpticalSubsystem
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
import wpimath.units

from wpimath.controller import RamseteController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint

from constants import AutoConstants

# 9982 - the intake falls to the front of the robot

class DriveSubsystem(commands2.Subsystem):

    opticalSubsystem: OpticalSubsystem = None
    ramsete: RamseteController = None
    voltConstraint: DifferentialDriveVoltageConstraint = None
    trajectoryConfig: TrajectoryConfig = None

    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()
        self.opticalSubsystem = OpticalSubsystem()
        # The motors on the left side of the drive.
        
        # phoenix5.VictorSPX(constants.DriveConstants.kLeftMotor1CanID),

        self.leftMotors = wpilib.MotorControllerGroup(
            wpilib.PWMSparkMax(constants.DriveConstants.kLeftMotorPort),
        )
    

        # The motors on the right side of the drive.
        self.rightMotors = wpilib.MotorControllerGroup(
            wpilib.PWMSparkMax(constants.DriveConstants.kRightMotorPort),
        )

        # The robot's drive
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotors, self.rightMotors)

        # The left-side drive encoder
        self.leftEncoder = wpilib.Encoder(
            constants.DriveConstants.kLeftEncoderPorts[0],
            constants.DriveConstants.kLeftEncoderPorts[1],
            constants.DriveConstants.kLeftEncoderReversed,
        )

        # The right-side drive encoder
        self.rightEncoder = wpilib.Encoder(
            constants.DriveConstants.kRightEncoderPorts[0],
            constants.DriveConstants.kRightEncoderPorts[1],
            constants.DriveConstants.kRightEncoderReversed,
        )

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightMotors.setInverted(True)

        # Sets the distance per pulse for the encoders
        self.leftEncoder.setDistancePerPulse(
            constants.DriveConstants.kEncoderDistancePerPulse
        )
        self.rightEncoder.setDistancePerPulse(
            constants.DriveConstants.kEncoderDistancePerPulse
        )
        
        self.leftEncoder.reset()
        self.rightEncoder.reset()
        self.gyro = navx.AHRS.create_spi()

        self.gyro.reset()
        iRot = self.gyro.getRotation2d()
        iPose = wpimath.geometry.Pose2d(0,0,iRot)
        self.odometry = wpimath.estimator.DifferentialDrivePoseEstimator(constants.AutoConstants.kDriveKinematics,self.gyro.getRotation2d(), 0, 0, iPose )
        self.leftEncoder.setDistancePerPulse(constants.AutoConstants.kEncoderDistancePerTick)
        # self.gyro = wpilib.ADXRS450_Gyro()
        
        self.tab = Shuffleboard.getTab("Drive")
        self.gyroWidget = self.tab.add("Gyro -> getAngle",0)
        self.headingRadians = self.tab.add("Heading(Rad)",0)
        self.headingWidget = self.tab.add("Heading",0)
        self.turnRateWidget = self.tab.add("TurnRate",0)
        self.poseRotationWidget = self.tab.add("PoseRotation",0)
        self.poseXWidget = self.tab.add("Pose X",0)
        self.poseYWidget = self.tab.add("Pose Y",0)
        self.leftEncoderWidget = self.tab.add("LeftEncoder",0)
        self.leftEncoderDistWidget = self.tab.add("LeftEncoderDist",0)
        self.leftEncoderDPPWidget = self.tab.add("LeftDistPerPulse",0)
        self.rightEncoderWidget = self.tab.add("RightEncoder",0)
        self.rightEncoderDistWidget = self.tab.add("RightEncoderDist",0)
        self.rightEncoderDPPWidget = self.tab.add("RightDistPerPulse",0)
        self.isThereCamPose = self.tab.add("IsThereCamPose", False)

        self.bluePoseX = self.tab.add("bluePoseX", 0.0)
        self.bluePoseY = self.tab.add("bluePoseY", 0.0)
        self.bluePoseRZ = self.tab.add("bluePoseRZ", 0.0)

        #self.photonposeestimator = photonlibpy

        AutoBuilder.configureRamsete(
            self.odometry.getEstimatedPosition, # Robot pose supplier
            self.odometry.resetPosition, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getCurrentSpeeds, # Current ChassisSpeeds supplier
            self, # Method that will drive the robot given ChassisSpeeds
            ReplanningConfig(), # Default path replanning config. See the API for the options here
            self.shouldFlipPath,
            self # Reference to this subsystem to set requirements
        )

        # blank constructor defaults to b:2.0 zeta:0.7
        self.ramsete = RamseteController()

        self.voltConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforwardMeters(
                kS=AutoConstants.ksVolts,
                kV=AutoConstants.kvVoltSecondsPerMeter,
                kA=AutoConstants.kaVoltSecondsSquaredPerMeter
            ),
            AutoConstants.kDriveKinematics,
            maxVoltage=10
        )

        self.trajectoryConfig = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )

        # not sure why these can't just be passed in as constructor parameters?
        self.trajectoryConfig.setKinematics(AutoConstants.kDriveKinematics)
        self.trajectoryConfig.addConstraint(self.voltConstraint)


    def periodic(self):
        self.odometry.update(self.gyro.getRotation2d(),self.leftEncoder.getDistance(),self.rightEncoder.getDistance())
        if self.opticalSubsystem.greenPose != None:
            # self.camPoseX.getEntry().setFloat(self.opticalSubsystem.bluePose.estimatedPose.X())
            # self.camPoseY.getEntry().setFloat(self.opticalSubsystem.bluePose.estimatedPose.Y())

            self.bluePoseX.getEntry().setFloat(self.opticalSubsystem.greenPose.estimatedPose.X())
            self.bluePoseY.getEntry().setFloat(self.opticalSubsystem.greenPose.estimatedPose.Y())
            self.bluePoseRZ.getEntry().setFloat(self.opticalSubsystem.greenPose.estimatedPose.rotation().z_degrees)
            timestamp = wpimath.units.seconds(self.opticalSubsystem.greenPose.timestampSeconds)
            estpose = self.opticalSubsystem.greenPose.estimatedPose
            self.odometry.addVisionMeasurement(
                estpose.toPose2d(),
                timestamp
            )
        estimatedPosition = self.odometry.getEstimatedPosition()
        self.gyroWidget.getEntry().setDouble(self.gyro.getAngle())
        self.headingWidget.getEntry().setDouble(self.getHeading())
        self.turnRateWidget.getEntry().setDouble(self.getTurnRate())
        self.leftEncoderWidget.getEntry().setDouble(self.leftEncoder.get())
        self.rightEncoderWidget.getEntry().setDouble(self.rightEncoder.get())
        self.poseRotationWidget.getEntry().setDouble(estimatedPosition.rotation().degrees())
        self.poseXWidget.getEntry().setDouble(estimatedPosition.X())
        self.poseYWidget.getEntry().setDouble(estimatedPosition.Y())
        self.leftEncoderDPPWidget.getEntry().setDouble(self.leftEncoder.getDistancePerPulse())
        self.rightEncoderDPPWidget.getEntry().setDouble(self.rightEncoder.getDistancePerPulse())

        self.leftEncoderDistWidget.getEntry().setFloat(self.leftEncoder.getDistance())
        self.rightEncoderDistWidget.getEntry().setFloat(self.rightEncoder.getDistance())

        self.isThereCamPose.getEntry().setBoolean(self.opticalSubsystem.bluePose != None)


    def arcadeDrive(self, fwd: float, rot: float):
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """


        self.drive.arcadeDrive(fwd, rot)

    def resetEncoders(self):
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getAverageEncoderDistance(self):
        """
        Gets the average distance of the two encoders.

        :returns: the average of the two encoder readings
        """
        return (self.leftEncoder.getDistance() + self.rightEncoder.getDistance()) / 2.0

    def getLeftEncoder(self) -> wpilib.Encoder:
        """
        Gets the left drive encoder.

        :returns: the left drive encoder
        """
        return self.leftEncoder

    def getRightEncoder(self) -> wpilib.Encoder:
        """
        Gets the right drive encoder.

        :returns: the right drive encoder
        """
        return self.rightEncoder

    def setMaxOutput(self, maxOutput: float):
        """
        Sets the max output of the drive. Useful for scaling the drive to drive more slowly.

        :param maxOutput: the maximum output to which the drive will be constrained
        """
        self.drive.setMaxOutput(maxOutput)

    def zeroHeading(self):
        """
        Zeroes the heading of the robot.
        """
        self.gyro.reset()


    def getPose2d(self) -> wpimath.geometry.Pose2d:
        """
        Returns the current Pose2d of the robot.  Based on PoseEstimator
        returns: postEstimator Pose2d
        """
        #pose = wpimath.geometry.Pose2d(self.getGyroRotation2d(),0,0)
        pose = wpimath.geometry.Pose2d(self.odometry.getEstimatedPosition())
        return pose


    def getHeading(self) -> float:
        """
        Returns the heading of the robot.
        :returns: the robot's heading in degrees, from 180 to 180
        """
        #return math.remainder(self.gyro.getAngle(), 180) * (
        #    -1 if constants.DriveConstants.kGyroReversed else 1
        #)

        # Convert Radian-based heading to degrees
        return self.gyro.getRotation2d().degrees()
        

    def getTurnRate(self):
        """
        Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (
            -1 if constants.DriveConstants.kGyroReversed else 1
        )

    def getLeftVelocity(self):
        return self.leftEncoder.getVelocity()
    
    def getRightVelocity(self):
        return self.leftEncoder.getVelocity()


    def getCurrentSpeeds(self):
        speeds = wpimath.kinematics.DifferentialDriveWheelSpeeds()
        return constants.AutoConstants.kDriveKinematics.toChassisSpeeds(speeds)
    

    def shouldFlipPath() -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed