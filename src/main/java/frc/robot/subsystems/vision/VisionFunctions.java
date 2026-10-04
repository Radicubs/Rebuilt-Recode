package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldManager;

import java.lang.reflect.Field;
import java.util.Objects;
import java.util.Optional;

public class VisionFunctions extends SubsystemBase {

  private static final Pose2d ZERO_POSE = new Pose2d();
  private static final int RED_HUB_TAG_ID = 10;
  private static final int BLUE_HUB_TAG_ID = 26;
  private static final double MIN_SHOOTING_DISTANCE_METERS = 1.95;
  private static final double MAX_SHOOTING_DISTANCE_METERS = 2.075;

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

  public static double getHubDistanceMeters() {
    Pose2d hubTag = FieldManager.getTagPose(getHubTagId());
    return hubTag.getTranslation().getDistance(getInstance().drivetrain.getPose().getTranslation());
  }

}
