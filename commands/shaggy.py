from commands2 import SequentialCommandGroup
from subsystems.drivesubsystem import DriveSubsystem

from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine

class Shaggy(SequentialCommandGroup):
    def __init__(self, drivesubsystem: DriveSubsystem):
        self.legs: DriveSubsystem = drivesubsystem
        
        routineAlpha = SysIdRoutine(
            SysIdRoutine.Config(1, 7, 3.5, None),
            SysIdRoutine.Mechanism(drivesubsystem.setVoltagesBoth, drivesubsystem.logMotors, drivesubsystem)
        )

        routineBeta = SysIdRoutine(
            SysIdRoutine.Config(1, 7, 7.5, None),
            SysIdRoutine.Mechanism(drivesubsystem.setVoltagesBoth, drivesubsystem.logMotors, drivesubsystem)
        )

        super().__init__(
            routineAlpha.dynamic(SysIdRoutine.Direction.kReverse),
            routineAlpha.dynamic(SysIdRoutine.Direction.kForward),
            routineBeta.quasistatic(SysIdRoutine.Direction.kReverse),
            routineBeta.quasistatic(SysIdRoutine.Direction.kForward)
        )
        return

    def isFinished(self) -> bool:
        return False