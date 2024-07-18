from commands2 import Subsystem, Command, cmd
from constants import IntakeConstants as IC
from rev import CANSparkMax, CANSparkLowLevel

class IntakeSubsystem(Subsystem):
    __upperMotor: CANSparkMax
    __lowerMotor: CANSparkMax

    __isRunning: bool = False
    __isRunningRev: bool = False

    def __init__(self) -> None:
        super().__init__()

        self.__upperMotor = CANSparkMax(IC.UPPER_ID, CANSparkLowLevel.MotorType.kBrushless)
        self.__lowerMotor = CANSparkMax(IC.LOWER_ID, CANSparkLowLevel.MotorType.kBrushless)
        #self.__lowerMotor.ControlType(CANSparkLowLevel.ControlType.kDutyCycle)
        #self.__upperMotor.ControlType(CANSparkLowLevel.ControlType.kDutyCycle)

    def setSpeed(self, mult: int):
        self.__lowerMotor.set(IC.ROLLER_DN_SPEED * mult)
        self.__upperMotor.set(IC.ROLLER_UP_SPEED * mult)
        #self.__lowerMotor.setVoltage(IC.ROLLER_DN_SPEED * mult)
        #self.__upperMotor.setVoltage(IC.ROLLER_UP_SPEED * mult)

    def runIntake(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(1), self
        )
    
    def runIntakeRev(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(-1), self
        )
    
    def stopIntake(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(0), self
        )