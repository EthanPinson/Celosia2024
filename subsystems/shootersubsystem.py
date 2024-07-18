from commands2 import Subsystem, Command, cmd
from constants import ShooterConstants as SC
from wpilib import Spark

class ShooterSubsystem(Subsystem):
    __innerMotor: Spark
    __outerMotor: Spark

    def __init__(self) -> None:
        super().__init__()

        self.__innerMotor = Spark(SC.INNER_ID)
        self.__outerMotor = Spark(SC.OUTER_ID)

    def setSpeed(self, speed: float):
        self.__outerMotor.set(speed)
        self.__innerMotor.set(speed)

    def setSpeeds(self, outer_speed: float, inner_speed: float):
        self.__outerMotor.set(outer_speed)
        self.__innerMotor.set(inner_speed)

    def runShooterAmp(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(SC.AMP_SPEED), self
        )

    def runShooter(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(SC.SHOOT_SPEED), self
        )
    
    def runShooterRev(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(-1.0 * SC.AMP_SPEED), self
        )
    
    def stopShooter(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(0.), self
        )