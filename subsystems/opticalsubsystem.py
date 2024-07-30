from commands2 import Subsystem
from robotpy_apriltag import AprilTagFieldLayout
from photonlibpy.photonCamera import PhotonCamera
from wpimath.units import degreesToRadians as torad
from wpimath.geometry import Transform3d, Rotation3d
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from constants import OpticalConstants as OC

class OpticalSubsystem(Subsystem):
    # from the center of the robot to the camera mount position
    BLU_CAM_OFFSET = Transform3d(0.1, 0.2, 0.5, Rotation3d(torad(16), torad(180), torad(-90)))
    GRN_CAM_OFFSET = Transform3d(0, 0, 0, Rotation3d())

    def __init__(self) -> None:
        super().__init__()

        self.FIELD_TAG_LAYOUT = AprilTagFieldLayout.loadField(OC.TAG_FIELD)
        
        # NOTE - robot poses will return as None if unsuccessful!
        # NOTE - a minimum of two apriltags in view are required for pose calculation!
        self.bluPose: EstimatedRobotPose = None
        self.grnPose: EstimatedRobotPose = None

        self.bluTargets: list[PhotonTrackedTarget] = []
        self.grnTargets: list[PhotonTrackedTarget] = []

        self._bluCam = PhotonCamera(OC.BLU_USB_NAME)
        self._grnCam = PhotonCamera(OC.GRN_USB_NAME)

        self._bluPoseEstimator = PhotonPoseEstimator(
            self.FIELD_TAG_LAYOUT, OC.POSE_STRATEGY, self._bluCam, self.BLU_CAM_OFFSET)
        self._grnPoseEstimator = PhotonPoseEstimator(
            self.FIELD_TAG_LAYOUT, OC.POSE_STRATEGY, self._grnCam, self.GRN_CAM_OFFSET)

        return None

    def periodic(self) -> None:
        bluResult = self._bluCam.getLatestResult()
        grnResult = self._grnCam.getLatestResult()

        self.bluPose = self._bluPoseEstimator.update(bluResult)
        self.grnPose = self._grnPoseEstimator.update(grnResult)

        self.bluTargets = bluResult.getTargets()
        self.grnTargets = grnResult.getTargets()

        return None