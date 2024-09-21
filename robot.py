#!/usr/bin/env python3

from commands2 import TimedCommandRobot, CommandScheduler
from wpilib import DataLogManager, DriverStation
from robotcontainer import RobotContainer
from networktables import NetworkTables

from constants import RoboRioConstants as RRc

class Robot(TimedCommandRobot):
    def robotInit(self):
        DataLogManager.start(period=RRc.LOG_FLUSH_PERIOD)
        DataLogManager.logNetworkTables(RRc.LOG_NT)

        self.autoCmd = None
        self.robotContainer = RobotContainer()

        NetworkTables.startClient(RRc.IP)

    # robotPeriodic is taken care of by TimedCommandRobot

    #def disabledInit(self): self.robotContainer.cleanup()

    def disabledPeriodic(self): pass

    def autonomousInit(self):
        self.autoCmd = self.robotContainer.getAutonomousCommand()
        if self.autoCmd is not None: self.autoCmd.schedule()

    def autonomousPeriodic(self): pass

    def teleopInit(self):
        DriverStation.startDataLog(DataLogManager.getLog())
        if self.autoCmd is not None: self.autoCmd.cancel()

    def teleopPeriodic(self): pass

    def testInit(self):
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self): pass

    def simulationInit(self): pass

    def simulationPeriodic(self): pass