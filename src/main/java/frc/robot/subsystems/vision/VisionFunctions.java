package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldManager;

import java.util.Objects;

public class VisionFunctions extends SubsystemBase {

  private static final int RED_HUB_TAG_ID = 10;
  private static final int BLUE_HUB_TAG_ID = 26;


  private static VisionFunctions instance;

  private final Drive drivetrain;
  private final Vision vision;

  // Initialization

  private VisionFunctions(Drive drivetrain) {
    this.drivetrain = Objects.requireNonNull(drivetrain);
    this.vision = Vision.getInstance();
  }

  public static void init(Drive drivetrain) {
    if (instance == null) {
      instance = new VisionFunctions(drivetrain);
    }
  }

  private static VisionFunctions getInstance() {
    if (instance == null) {
      throw new IllegalStateException("VisionFunctions not initialized. Call init(...) first.");
    }
    return instance;
  }

  public static int getHubTagId() {
    return FieldManager.isRedAlliance() ? RED_HUB_TAG_ID : BLUE_HUB_TAG_ID;
  }

  public static boolean targetAvailable() {
    return getInstance().vision.getBestTagId() != -1;
  }

  public static Pose2d getHubTargetPose() {
    Pose2d tagPose = FieldManager.getTagPose(getHubTagId());
    // Tag-relative negative X points inward on both alliances.
    return tagPose.plus(
        new Transform2d(new Translation2d(-VisionConstants.HUB_OFFSET_METERS, 0.0), Rotation2d.kZero));
  }

  public static double getHubDistanceMeters() {
    return getHubTargetPose().getTranslation()
        .getDistance(getInstance().drivetrain.getPose().getTranslation());
  }

}
