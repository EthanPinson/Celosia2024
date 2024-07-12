
import math
from wpimath.kinematics import DifferentialDriveKinematics
from wpilib import SPI

class DriveConstants:
    kLeftFrontMotorPort = 4
    kLeftRearMotorPort = 3

    kRightFrontMotorPort = 2
    kRightRearMotorPort = 1

    kLeftEncoderPorts = (9, 8)
    kRightEncoderPorts = (7, 6)
    kLeftEncoderReversed = False
    kRightEncoderReversed = True

    # Encoder counts per revolution/rotation.
    kEncoderCPR = 360
    kWheelDiameterMeters = 0.152

    # Assumes the encoders are directly mounted on the wheel shafts
    kEncoderDistancePerPulse = (kWheelDiameterMeters * math.pi) / kEncoderCPR

    kGyroReversed = False

    kStabilizationP = 1
    kStabilizationI = 0.5
    kStabilizationD = 0

    #kTurnP = .01
    kTurnP = 0.5
    #kTurnP = .004
    kTurnI = 0
    kTurnD = 0.375

    kMaxTurnRateDegPerS = 100
    kMaxTurnAccelerationDegPerSSquared = 300

    kTurnToleranceDeg = 5
    kTurnRateToleranceDegPerS = 10  # degrees per second

    # Radian conversions of the above
    kMaxTurnRateRadPerS = kMaxTurnRateDegPerS * math.pi / 180.0
    kMaxTurnAccelerationRadPerSSquared = kMaxTurnAccelerationDegPerSSquared * math.pi / 180.0

    kTurnToleranceRad = kTurnToleranceDeg * math.pi / 180.0
    kTurnRateToleranceRadPerS = kTurnRateToleranceDegPerS * math.pi / 180.0


class AutoConstants:

    # In meters, distance between wheels on each side of robot.
    kTrackWidthMeters = 0.60
    kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)

    # Encoder counts per revolution/rotation.
    kEncoderCPR = 360
    kWheelDiameterMeters = 0.152
    kEncoderDistancePerTick = math.pi * kWheelDiameterMeters / kEncoderCPR

    # NOTE: Please do NOT use these values on your robot. Rather, characterize your
    # drivetrain using the FRC Characterization tool. These are for demo purposes
    # only!
    ksVolts = 2.155
    kvVoltSecondsPerMeter = 2.5377
    kaVoltSecondsSquaredPerMeter = 1.1553

    # The P gain for our turn controllers.
    kPDriveVel = 1

    # The max velocity and acceleration for our autonomous.
    kMaxSpeedMetersPerSecond = .5
    kMaxAccelerationMetersPerSecondSquared = .5

    # Baseline values for a RAMSETE follower in units of meters
    # and seconds. These are recommended, but may be changes if wished.
    kRamseteB = 2
    kRamseteZeta = 0.7

    # The number of motors on the robot.
    kDrivetrainMotorCount = 4

class OIConstants:
    kDriverControllerPort = 0
    kDriverRightBumper = 6
    kDriverLeftBumper = 5
    kDriverXbutton = 3
    kDriverYbutton =  4

class OpticalConstants:
    # POSE_STRATEGY
    # TAG_LAYOUT
    SKEPTICISM: tuple[float, float, float] = (0.1, 0.1, 0.1)

class TrajectoryConstants:
    MAX_VOLTAGE = 10
    MAX_VELOCITY: float = 0.5
    MAX_ACCELERATION: float = 0.5

class ControllerConstants:
    B: float = 2.0
    ZETA: float = 0.7

    STATIC_GAIN: float = 2.155 # 9982: 2.03-2.6386
    VELOCITY_GAIN: float = 2.5377 # 9982: 2.1313-2.7174
    ACCELERATION_GAIN: float = 1.1553 # 9982: 1.7129-2.7601

class GyroConstants:
    PORT: SPI.Port = SPI.Port.kMXP
    BITRATE: int = 500000
    UPADTE_RATE: int = 60 # Hz
    SKEPTICISM: tuple[float, float, float] = (0.02, 0.02, 0.01)

class ShooterConstants: # Spark x2
    INNER_SPEED: float = 0.9
    OUTER_SPEED: float = 1.0

    INNER_ID: int = 0
    OUTER_ID: int = 1

class FeederConstants: # Spark x1
    NOMINAL_SPEED: float = -0.5

    ID: int = 2

class IntakeConstants: # SparkMAX x2
    ROLLER_UP_SPEED: float = 0.25
    ROLLER_DN_SPEED: float = 0.20

    UPPER_ID: int = 1 #3
    LOWER_ID: int = 6 #4