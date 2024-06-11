from commands2 import Command, Subsystem
from wpimath.controller import RamseteController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import Trajectory
from typing import Callable
from wpimath.geometry import Pose2d
from wpimath.kinematics import (
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
    ChassisSpeeds,
)
from wpimath.controller import PIDController
from wpilib import Timer


class RamseteCommand(Command):
    m_timer = Timer()
    m_usePID: bool
    m_trajectory: Trajectory
    m_pose: Callable[[], Pose2d]
    m_follower: RamseteController
    m_feedforward: SimpleMotorFeedforwardMeters
    m_kinematics: DifferentialDriveKinematics
    m_speeds: Callable[[], DifferentialDriveWheelSpeeds]
    m_leftController: PIDController
    m_rightController: PIDController
    m_output: Callable[[float, float], None]
    m_prevSpeeds: DifferentialDriveWheelSpeeds = DifferentialDriveWheelSpeeds()
    m_prevTime: float
    def __init__(
        self,
        trajectory: Trajectory,
        pose: Callable[[], Pose2d],
        controller: RamseteController,
        feedforward: SimpleMotorFeedforwardMeters,
        kinematics: DifferentialDriveKinematics,
        wheelSpeeds: Callable[[], DifferentialDriveWheelSpeeds],
        leftController: PIDController,
        rightController: PIDController,
        outputVolts: Callable[[float, float], None],
        *requirements: Subsystem,
    ):
        super().__init__()

        assert callable(pose)
        assert callable(wheelSpeeds)
        assert callable(outputVolts)

        self.m_trajectory = trajectory
        self.m_pose = pose
        self.m_follower = controller
        self.m_feedforward = feedforward
        self.m_kinematics = kinematics
        self.m_speeds = wheelSpeeds
        self.m_leftController = leftController
        self.m_rightController = rightController
        self.m_output = outputVolts

        self.m_usePID = True
        self.addRequirements(*requirements)

    def initialize(self):
        self.m_prevTime = -1
        initialState = self.m_trajectory.sample(0)
        self.m_prevSpeeds = self.m_kinematics.toWheelSpeeds(
            ChassisSpeeds(
                initialState.velocity(),
                0,
                initialState.curvature * initialState.velocity,
            )
        )
        self.m_timer.restart()
        if self.m_usePID:
            self.m_leftController.reset()
            self.m_rightController.reset()

    def execute(self):
        curTime: float = self.m_timer.get()
        dt: float = curTime - self.m_prevTime

        if self.m_prevTime < 0:
            self.m_output(0.0, 0.0)
            self.m_prevTime = curTime

        targetWheelSpeeds = self.m_kinematics.toWheelSpeeds(
            self.m_follower.calculate(self.m_pose(), self.m_trajectory.sample(curTime))
        )

        leftSpeedSetpoint = targetWheelSpeeds.left
        rightSpeedSetpoint = targetWheelSpeeds.right

        leftOutput: float
        rightOutput: float
        
        if self.m_usePID:
            leftFeedforward: float = self.m_feedforward.calculate(
                leftSpeedSetpoint, (leftSpeedSetpoint - self.m_prevSpeeds.left) / dt)
            rightFeedforward: float = self.m_feedforward.calculate(
                rightSpeedSetpoint, (rightSpeedSetpoint - self.m_prevSpeeds.right) / dt)
            
            leftOutput = leftFeedforward + self.m_leftController.calculate(self.m_speeds().left, leftSpeedSetpoint)
            rightOutput = rightFeedforward + self.m_rightController.calculate(self.m_speeds().right, rightSpeedSetpoint)
        else:
            leftOutput = leftSpeedSetpoint
            rightOutput = rightSpeedSetpoint
        
        self.m_output(leftOutput, rightOutput)
        self.m_prevSpeeds = targetWheelSpeeds
        self.m_prevTime = curTime
    
    def end(self, interrupted: bool):
        self.m_timer.stop()
        if interrupted:
            self.m_output(0.0, 0.0)

    def isFinished(self):
        return self.m_timer.hasElapsed(self.m_trajectory.totalTime())