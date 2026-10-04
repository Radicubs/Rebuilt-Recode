package frc.robot.subsystems.vision;

import frc.robot.constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.util.FieldManager;

import java.util.Optional;

// AprilTag localization; fuses multi-tag estimates into the pose estimator. Real cameras only.
public class Vision extends SubsystemBase {

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = FieldManager.getLayout();

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        return instance;
    }

    private final PhotonCamera camera0 = new PhotonCamera("orange");
    private final PhotonPoseEstimator estimator0 = new PhotonPoseEstimator(APRIL_TAG_LAYOUT,
            new Transform3d(VisionConstants.camera_0_OffsetX, VisionConstants.camera_0_OffsetY,
                    VisionConstants.camera_0_OffsetZ, new Rotation3d(Rotation2d.k180deg)));
    private PhotonPipelineResult latestResult0;
    private EstimatedRobotPose latestEstimate0;

    private final PhotonCamera camera1 = new PhotonCamera("juice");
    private final PhotonPoseEstimator estimator1 = new PhotonPoseEstimator(APRIL_TAG_LAYOUT,
            new Transform3d(VisionConstants.camera_1_OffsetX, VisionConstants.camera_1_OffsetY,
                    VisionConstants.camera_1_OffsetZ, new Rotation3d(Rotation2d.kCCW_90deg)));
    private PhotonPipelineResult latestResult1;
    private EstimatedRobotPose latestEstimate1;

    private Vision() {
        VisionLogger.publish(this);
    }

    /** Prefer orange's best known field tag, then fall back to juice. */
    public int getBestTagId() {
        return selectTagId(latestResult0, camera0.isConnected(),
                latestResult1, camera1.isConnected(), Timer.getFPGATimestamp());
    }

    static int selectTagId(PhotonPipelineResult result0, boolean connected0,
            PhotonPipelineResult result1, boolean connected1, double nowSeconds) {
        int tag0 = tagId(result0, connected0, nowSeconds);
        return tag0 != -1 ? tag0 : tagId(result1, connected1, nowSeconds);
    }

    private static int tagId(PhotonPipelineResult result, boolean connected, double nowSeconds) {
        if (!connected || result == null || !isFresh(result.getTimestampSeconds(), nowSeconds)) return -1;
        return result.getTargets().stream()
                .filter(target -> APRIL_TAG_LAYOUT.getTagPose(target.getFiducialId()).isPresent())
                .mapToInt(target -> target.getFiducialId()).findFirst().orElse(-1);
    }

    private static boolean isFresh(double timestampSeconds, double nowSeconds) {
        double age = nowSeconds - timestampSeconds;
        return Double.isFinite(age) && age >= 0.0 && age <= VisionConstants.maxResultAgeSeconds;
    }

    /** Fresh raw camera pose, before drivetrain fusion, for camera-first helpers and diagnostics. */
    public Optional<EstimatedRobotPose> getLatestEstimatedPose() {
        double now = Timer.getFPGATimestamp();
        EstimatedRobotPose estimate0 = camera0.isConnected() && latestEstimate0 != null
                && isFresh(latestEstimate0.timestampSeconds, now) ? latestEstimate0 : null;
        EstimatedRobotPose estimate1 = camera1.isConnected() && latestEstimate1 != null
                && isFresh(latestEstimate1.timestampSeconds, now) ? latestEstimate1 : null;
        if (estimate0 == null) return Optional.ofNullable(estimate1);
        if (estimate1 == null) return Optional.of(estimate0);
        return Optional.of(estimate0.timestampSeconds >= estimate1.timestampSeconds ? estimate0 : estimate1);
    }

    boolean cam0Connected() { return camera0.isConnected(); }
    boolean cam1Connected() { return camera1.isConnected(); }

    @Override
    public void periodic() {
        latestResult0 = process(camera0, estimator0, latestResult0);
        latestResult1 = process(camera1, estimator1, latestResult1);
        VisionLogger.log(this);
    }

    private PhotonPipelineResult process(PhotonCamera camera, PhotonPoseEstimator estimator, PhotonPipelineResult last) {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (camera == camera0) latestEstimate0 = null;
            else latestEstimate1 = null;
            if (!result.hasTargets() || !isFresh(result.getTimestampSeconds(), Timer.getFPGATimestamp())) {
                last = null;
                continue;
            }
            last = result;
            Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
            estimate.ifPresent(est -> {
                if (camera == camera0) latestEstimate0 = est;
                else latestEstimate1 = est;
                Drive.getInstance().addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
            });
        }
        return last;
    }
}
