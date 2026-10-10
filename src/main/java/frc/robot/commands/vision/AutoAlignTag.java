package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionFunctions;
import frc.robot.util.FieldManager;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Aims the rear-facing shooter at the alliance hub, optionally while driving. */
public class AutoAlignTag extends Command {

  private final Drive drivetrain;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final boolean finishWhenAligned;
  private final PIDController aimController = new PIDController(VisionConstants.lockKP, 0.0, 0.0);

  private boolean aligned;

  /** Stationary alignment command that finishes when aligned. */
  public AutoAlignTag(Drive drivetrain) {
    this(drivetrain, () -> 0.0, () -> 0.0, true);
  }

  /** Hold-to-align command. Joysticks control translation; alignment owns rotation. */
  public AutoAlignTag(Drive drivetrain, DoubleSupplier translationX,
      DoubleSupplier translationY) {
    this(drivetrain, translationX, translationY, false);
  }

  private AutoAlignTag(Drive drivetrain, DoubleSupplier translationX,
      DoubleSupplier translationY, boolean finishWhenAligned) {
    this.drivetrain = drivetrain;
    this.translationX = translationX;
    this.translationY = translationY;
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
    aligned = Math.abs(headingError.getDegrees()) <= VisionConstants.ALIGNMENT_TOLERANCE_DEGREES;

    // This command requires the drivetrain, suspending TeleopDrive while active.
    // Always use alignment rotation, even when the driver moves the right stick.
    double rotationInput = MathUtil.clamp(
          aimController.calculate(robotPose.getRotation().getRadians(), targetHeading.getRadians()),
          -VisionConstants.lockOnMaxSpeed, VisionConstants.lockOnMaxSpeed);
    // Keep small corrections until aligned; an output deadband would stop short of the tolerance.
    if (aligned) {
      rotationInput = 0.0;
    }

    double allianceSign = FieldManager.isRedAlliance() ? -1.0 : 1.0;
    drivetrain.drive(
        new Translation2d(
            allianceSign * MathUtil.applyDeadband(translationX.getAsDouble(), DriveConstants.joystickDeadband) * DriveConstants.maxSpeed,
            allianceSign * MathUtil.applyDeadband(translationY.getAsDouble(), DriveConstants.joystickDeadband) * DriveConstants.maxSpeed),
        rotationInput * DriveConstants.maxAngularVelocity,
        true,
        false);

    Logger.recordOutput("Drive/RotToTag/Active", true);
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

  public boolean isAligned() {
    return aligned;
  }

  private Pose2d getTargetPose() {
    return VisionFunctions.getHubTargetPose();
  }
}
