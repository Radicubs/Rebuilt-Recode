package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

/** Small field-state helper for alliance lookups and dashboard field visualization. */
public class FieldManager extends SubsystemBase {
  private static final double AUTO_SHIFT_SECONDS = 20.0;
  private static final double TRANSITION_SECONDS = 10.0;
  private static final double ALLIANCE_SHIFT_SECONDS = 25.0;
  private static final double END_GAME_SECONDS = 30.0;

  private static final double END_GAME_CUTOFF_SECONDS = END_GAME_SECONDS;
  private static final double SHIFT_4_CUTOFF_SECONDS =
      END_GAME_CUTOFF_SECONDS + ALLIANCE_SHIFT_SECONDS;
  private static final double SHIFT_3_CUTOFF_SECONDS =
      SHIFT_4_CUTOFF_SECONDS + ALLIANCE_SHIFT_SECONDS;
  private static final double SHIFT_2_CUTOFF_SECONDS =
      SHIFT_3_CUTOFF_SECONDS + ALLIANCE_SHIFT_SECONDS;
  private static final double SHIFT_1_CUTOFF_SECONDS =
      SHIFT_2_CUTOFF_SECONDS + ALLIANCE_SHIFT_SECONDS;
  private static final double TELEOP_MATCH_SECONDS = SHIFT_1_CUTOFF_SECONDS + TRANSITION_SECONDS;

  private static final AprilTagFieldLayout layout = 
    AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    
  private static FieldManager mInstance;

  private final Field2d field = new Field2d();

  private FieldManager() {
    SmartDashboard.putData("Field", field);
  }

  public static FieldManager getInstance() {
    if (mInstance == null) {
      mInstance = new FieldManager();
    }
    return mInstance;
  }

  public static AprilTagFieldLayout getLayout() {
    return layout;
  }

  public static Pose2d getTagPose(int tag) {
    return layout.getTagPose(tag)
        .orElseThrow(() -> new IllegalArgumentException("Unknown AprilTag ID: " + tag)).toPose2d();
  }

  public Field2d getField() {
    return field;
  }

  public void setTargetPose(Pose2d pose) {
    field.getObject("TargetPose").setPose(pose);
  }

  public void setActivePath(List<Pose2d> poses) {
    field.getObject("ActivePath").setPoses(poses);
  }

  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public static boolean isRedAlliance() {
    return getAlliance() == Alliance.Red;
  }

  public static boolean isAlliancePresent() {
    return DriverStation.getAlliance().isPresent();
  }

  public double getTimeLeftInShift() {
    return getCurrentShiftState().timeLeftInShiftSeconds();
  }

  public String getShiftType() {
    return getCurrentShiftState().shiftType();
  }

  private ShiftState getCurrentShiftState() {
    return determineShiftState(
        DriverStation.isAutonomousEnabled(),
        DriverStation.isTeleopEnabled(),
        DriverStation.getMatchTime(),
        DriverStation.getGameSpecificMessage());
  }

  static ShiftState determineShiftState(
      boolean autonomousEnabled, boolean teleopEnabled, double matchTimeSeconds, String gameData) {
    if (autonomousEnabled) {
      return new ShiftState("Auto Shift", clampTimeLeft(matchTimeSeconds, AUTO_SHIFT_SECONDS));
    }

    if (!teleopEnabled) {
      return new ShiftState("No Shift", 0.0);
    }

    double matchTime = clampTimeLeft(matchTimeSeconds, TELEOP_MATCH_SECONDS);
    if (matchTime > SHIFT_1_CUTOFF_SECONDS) {
      return new ShiftState(
          "Transition Shift", clampTimeLeft(matchTime - SHIFT_1_CUTOFF_SECONDS, TRANSITION_SECONDS));
    }

    Optional<Alliance> shift1InactiveAlliance = getShift1InactiveAlliance(gameData);
    if (matchTime > SHIFT_2_CUTOFF_SECONDS) {
      return createAllianceShiftState(
          getActiveAllianceForShift(shift1InactiveAlliance, 1), matchTime - SHIFT_2_CUTOFF_SECONDS);
    }
    if (matchTime > SHIFT_3_CUTOFF_SECONDS) {
      return createAllianceShiftState(
          getActiveAllianceForShift(shift1InactiveAlliance, 2), matchTime - SHIFT_3_CUTOFF_SECONDS);
    }
    if (matchTime > SHIFT_4_CUTOFF_SECONDS) {
      return createAllianceShiftState(
          getActiveAllianceForShift(shift1InactiveAlliance, 3), matchTime - SHIFT_4_CUTOFF_SECONDS);
    }
    if (matchTime > END_GAME_CUTOFF_SECONDS) {
      return createAllianceShiftState(
          getActiveAllianceForShift(shift1InactiveAlliance, 4), matchTime - END_GAME_CUTOFF_SECONDS);
    }

    return new ShiftState("End Game", matchTime);
  }

  private static ShiftState createAllianceShiftState(
      Optional<Alliance> activeAlliance, double timeLeftInShiftSeconds) {
    if (activeAlliance.isEmpty()) {
      return new ShiftState(
          "Unknown Shift", clampTimeLeft(timeLeftInShiftSeconds, ALLIANCE_SHIFT_SECONDS));
    }

    String shiftType = activeAlliance.get() == Alliance.Red ? "Red Shift" : "Blue Shift";
    return new ShiftState(
        shiftType, clampTimeLeft(timeLeftInShiftSeconds, ALLIANCE_SHIFT_SECONDS));
  }

  static Optional<Alliance> getShift1InactiveAlliance(String gameData) {
    if (gameData == null || gameData.isEmpty()) {
      return Optional.empty();
    }

    // REBUILT game data reports the alliance that scored more FUEL in AUTO, or the alliance
    // selected by FMS after an AUTO tie. That alliance's HUB is inactive in SHIFT 1.
    return switch (Character.toUpperCase(gameData.charAt(0))) {
      case 'R' -> Optional.of(Alliance.Red);
      case 'B' -> Optional.of(Alliance.Blue);
      default -> Optional.empty();
    };
  }

  private static Optional<Alliance> getActiveAllianceForShift(
      Optional<Alliance> shift1InactiveAlliance, int shiftNumber) {
    if (shiftNumber % 2 == 1) {
      return flipAlliance(shift1InactiveAlliance);
    }

    return shift1InactiveAlliance;
  }

  private static Optional<Alliance> flipAlliance(Optional<Alliance> alliance) {
    if (alliance.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(alliance.get() == Alliance.Red ? Alliance.Blue : Alliance.Red);
  }

  private static double clampTimeLeft(double timeLeftSeconds, double maxTimeSeconds) {
    return Math.max(0.0, Math.min(timeLeftSeconds, maxTimeSeconds));
  }

  @Override
  public void periodic() {
    ShiftState shiftState = getCurrentShiftState();
    field.setRobotPose(Drive.getInstance().getPose());
    SmartDashboard.putNumber("Shift/Time Left In Shift", shiftState.timeLeftInShiftSeconds());
    SmartDashboard.putString("Shift/Shift Type", shiftState.shiftType());
  }

  record ShiftState(String shiftType, double timeLeftInShiftSeconds) {}
}
