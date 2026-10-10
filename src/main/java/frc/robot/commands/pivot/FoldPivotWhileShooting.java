package frc.robot.commands.pivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.pivot.Pivot;

/** Folds to halfway while shooting, then brakes until the shot ends. */
public class FoldPivotWhileShooting extends Command {
  private final Pivot pivot;
  private final double speed;
  private boolean reachedFoldPosition;

  public FoldPivotWhileShooting(Pivot pivot, double speed) {
    this.pivot = pivot;
    this.speed = speed;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    reachedFoldPosition = false;
    execute();
  }

  @Override
  public void execute() {
    // Folding inward decreases encoder position. Latch the stop to avoid
    // restarting against the limit due to small sensor changes or arm motion.
    reachedFoldPosition |= pivot.getPosition() <= PivotConstants.shotFoldPosition;
    pivot.setSpeed(reachedFoldPosition ? 0.0 : speed);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0.0);
  }
}
