package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionFunctions;
import frc.robot.util.FieldManager;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Aims the rear-facing shooter at the alliance hub, optionally while driving. */
public class AutoAlignTag extends Command {

  private static final double HUB_OFFSET_METERS = 0.60365;
  private static final double ALIGNMENT_TOLERANCE_DEGREES = 0.2;

  private final Drive drivetrain;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;
  private final boolean finishWhenAligned;
  private final PIDController aimController = new PIDController(DriveConstants.lockKP, 0.0, 0.0);

  private boolean aligned;

  /** Stationary alignment command that finishes when aligned. */
  public AutoAlignTag(Drive drivetrain) {
    this(drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, true);
  }

  /** Hold-to-align command. Inputs are normalized joystick axes, matching TeleopDrive. */
  public AutoAlignTag(Drive drivetrain, DoubleSupplier translationX,
      DoubleSupplier translationY, DoubleSupplier rotation) {
    this(drivetrain, translationX, translationY, rotation, false);
  }

  private AutoAlignTag(Drive drivetrain, DoubleSupplier translationX,
      DoubleSupplier translationY, DoubleSupplier rotation, boolean finishWhenAligned) {
    this.drivetrain = drivetrain;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.finishWhenAligned = finishWhenAligned;
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    aimController.reset();
    aligned = false;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getPose();
    Pose2d targetPose = getTargetPose();
    Rotation2d targetHeading = targetPose.getTranslation()
        .minus(robotPose.getTranslation()).getAngle().plus(Rotation2d.k180deg);
    Rotation2d headingError = targetHeading.minus(robotPose.getRotation());
    aligned = Math.abs(headingError.getDegrees()) <= ALIGNMENT_TOLERANCE_DEGREES;

    double rotationInput = rotation.getAsDouble();
    boolean aiming = rotationInput == 0.0;
    if (aiming) {
      rotationInput = MathUtil.clamp(
          aimController.calculate(robotPose.getRotation().getRadians(), targetHeading.getRadians()),
          -DriveConstants.lockOnMaxSpeed, DriveConstants.lockOnMaxSpeed);
      // Keep small corrections until aligned; an output deadband would stop short of the tolerance.
      if (aligned) {
        rotationInput = 0.0;
      }
    } else {
      // Manual rotation takes priority; resume tracking when the stick returns to zero.
      aimController.reset();
    }

    double allianceSign = FieldManager.isRedAlliance() ? -1.0 : 1.0;
    drivetrain.drive(
        new Translation2d(
            allianceSign * translationX.getAsDouble() * DriveConstants.maxSpeed,
            allianceSign * translationY.getAsDouble() * DriveConstants.maxSpeed),
        rotationInput * DriveConstants.maxAngularVelocity,
        true,
        false);

    Logger.recordOutput("Drive/RotToTag/Active", aiming);
    Logger.recordOutput("Drive/RotToTag/Aligned", aligned);
    Logger.recordOutput("Drive/RotToTag/TargetPose", targetPose);
    Logger.recordOutput("Drive/RotToTag/TargetAngleRad", targetHeading.getRadians());
    Logger.recordOutput("Drive/RotToTag/ErrorRad", headingError.getRadians());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    Logger.recordOutput("Drive/RotToTag/Active", false);
  }

  @Override
  public boolean isFinished() {
    return finishWhenAligned && aligned;
  }

  private Pose2d getTargetPose() {
    Pose2d tagPose = FieldManager.getTagPose(VisionFunctions.getHubTagId());
    // Tag-relative negative X points inward on both alliances.
    return tagPose.plus(
        new Transform2d(new Translation2d(-HUB_OFFSET_METERS, 0.0), Rotation2d.kZero));
  }
}
