from commands2 import Subsystem
from robotpy_apriltag import AprilTagField
from photonlibpy.photonCamera import PhotonCamera
from wpimath.geometry import Transform3d, Rotation3d
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy


class OpticalSubsystem(Subsystem):
    STRATEGY: PoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    FIELD_TAGS: AprilTagField = AprilTagField.k2024Crescendo

    BLUE_CAMERA_OFFSET: Transform3d = Transform3d(-10, -10, 30, Rotation3d(0, 3.14/4, -3.14/2))
    GREEN_CAMERA_OFFSET: Transform3d = Transform3d(0, 0, 0, Rotation3d())

    BLUE_CAMERA_NAME: str = "blueCamera"
    GREEN_CAMERA_NAME: str = "greenCamera"

    bluePose: EstimatedRobotPose | None = None
    greenPose: EstimatedRobotPose | None = None

    blueTargets: list[PhotonTrackedTarget] = []
    greenTargets: list[PhotonTrackedTarget] = []

    __bluePoseEstimator: PhotonPoseEstimator = None
    __greenPoseEstimator: PhotonPoseEstimator = None

    __blueCamera: PhotonCamera = None
    __greenCamera: PhotonCamera = None

    def __init__(self) -> None:
        super().__init__()

        self.__blueCamera: PhotonCamera = PhotonCamera(self.BLUE_CAMERA_NAME)
        self.__greenCamera: PhotonCamera = PhotonCamera(self.GREEN_CAMERA_NAME)

        self.__bluePoseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
            self.FIELD_TAGS, self.STRATEGY, self.__blueCamera, self.BLUE_CAMERA_OFFSET
        )
        self.__greenPoseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
            self.FIELD_TAGS, self.STRATEGY, self.__greenCamera, self.GREEN_CAMERA_OFFSET
        )

        return None

    def periodic(self) -> None:
        blueResult: PhotonPipelineResult = self.__blueCamera.getLatestResult()
        greenResult: PhotonPipelineResult = self.__greenCamera.getLatestResult()

        self.bluePose = self.__bluePoseEstimator.update(blueResult)
        self.greenPose = self.__greenPoseEstimator.update(greenResult)

        self.blueTargets = blueResult.getTargets()
        self.greenTargets = greenResult.getTargets()

        print(self.bluePose)

        return None