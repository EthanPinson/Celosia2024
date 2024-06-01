from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from wpimath.geometry import Transform3d, Rotation3d
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy

class OpticalSubsystem(Subsystem):
    STRATEGY: PoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    FIELD_TAGS: AprilTagField = AprilTagField.k2024Crescendo

    # from the center of the robot to the camera mount position
    BLUE_CAMERA_OFFSET: Transform3d = Transform3d(0, 0, 0, Rotation3d())
    GREEN_CAMERA_OFFSET: Transform3d = Transform3d(0, 0, 0, Rotation3d())

    BLUE_CAMERA_NAME: str = "blueCamera"
    GREEN_CAMERA_NAME: str = "greenCamera"

    FIELD_TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(FIELD_TAGS)

    # NOTE - robot poses will return as None if unsuccessful!
    # NOTE - a minimum of two apriltags in view are required for pose calculation!
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
            self.FIELD_TAG_LAYOUT, self.STRATEGY, self.__blueCamera, self.BLUE_CAMERA_OFFSET
        )
        self.__greenPoseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
            self.FIELD_TAG_LAYOUT, self.STRATEGY, self.__greenCamera, self.GREEN_CAMERA_OFFSET
        )

        return None

    def periodic(self) -> None:
        blueResult: PhotonPipelineResult = self.__blueCamera.getLatestResult()
        greenResult: PhotonPipelineResult = self.__greenCamera.getLatestResult()

        self.bluePose = self.__bluePoseEstimator.update(blueResult)
        self.greenPose = self.__greenPoseEstimator.update(greenResult)

        self.blueTargets = blueResult.getTargets()
        self.greenTargets = greenResult.getTargets()

        return None