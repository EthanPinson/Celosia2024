from commands2 import Subsystem, Command
import commands2.cmd
from constants import ShooterConstants as SC
from wpilib import Spark

class ShooterSubsystem(Subsystem):
    __innerMotor: Spark
    __outerMotor: Spark

    def __init__(self) -> None:
        super().__init__()

        self.__innerMotor = Spark(SC.INNER_ID)
        self.__outerMotor = Spark(SC.OUTER_ID)

    def setSpeed(self):
        self.__innerMotor.set(SC.INNER_SPEED)
        self.__outerMotor.set(SC.OUTER_SPEED)

    def stop(self):
        self.__innerMotor.set(0.)
        self.__outerMotor.set(0.)

    def runShooter(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.setSpeed()
        )
    
    def stopShooter(self) -> Command:
        return commands2.cmd.runOnce(
            lambda: self.stop()
        )