from commands2 import Subsystem, Command, cmd
from constants import AmpConstants as AC
from wpilib import PWMVictorSPX

class AmpSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.__motor = PWMVictorSPX(AC.MOTOR_ID)

    def setSpeed(self, speed: float) -> Command:
        return cmd.runOnce(lambda: self.__motor.set(speed), self)