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

class DriveSubsystem(commands2.Subsystem):

    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()
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
        self.gyroWidget = self.tab.add("Gyro",0)
        self.headingRadians = self.tab.add("Heading(Rad)",0)
        self.headingWidget = self.tab.add("Heading",0)
        self.turnRateWidget = self.tab.add("TurnRate",0)
        self.poseRotationWidget = self.tab.add("PoseRotation",0)
        self.poseXWidget = self.tab.add("Pose X",0)
        self.poseYWidget = self.tab.add("Pose Y",0)
        self.leftEncoderWidget = self.tab.add("LeftEncoder",0)
        self.leftEncoderDistWidget = self.tab.add("LeftEncoder",0)
        self.leftEncoderDPPWidget = self.tab.add("LeftDistPerPulse",0)
        self.rightEncoderWidget = self.tab.add("RightEncoder",0)
        self.rightEncoderDistWidget = self.tab.add("RightEncoder",0)
        self.rightEncoderDPPWidget = self.tab.add("RightDistPerPulse",0)

        #self.photonposeestimator = photonlibpy

        pathplannerlib.auto.AutoBuilder.configureRamsete(
            self.odometry.getEstimatedPosition, # Robot pose supplier
            self.odometry.resetPosition, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getCurrentSpeeds, # Current ChassisSpeeds supplier
            self, # Method that will drive the robot given ChassisSpeeds
            ReplanningConfig(), # Default path replanning config. See the API for the options here
            self.shouldFlipPath,
            self # Reference to this subsystem to set requirements
        )


    def periodic(self):
        self.odometry.update(self.gyro.getRotation2d(),self.leftEncoder.getDistance(),self.rightEncoder.getDistance())
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
    

    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed