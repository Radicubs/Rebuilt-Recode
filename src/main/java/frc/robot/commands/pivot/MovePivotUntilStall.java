package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.pivot.Pivot;

/** Runs the pivot until sustained end-stop current is detected. */
public class MovePivotUntilStall extends Command {
    private final Pivot pivot;
    private final double speed;
    private final Timer startupTimer = new Timer();
    private final Timer stallTimer = new Timer();

    public MovePivotUntilStall(Pivot pivot, double speed) {
        this.pivot = pivot;
        this.speed = speed;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        startupTimer.restart();
        stallTimer.stop();
        stallTimer.reset();
        pivot.setSpeed(speed);
    }

    @Override
    public void execute() {
        pivot.setSpeed(speed);
        if (startupTimer.hasElapsed(PivotConstants.stallStartupDelaySeconds)
                && pivot.getCurrentAmps() >= PivotConstants.stallCurrentAmps) {
            stallTimer.start();
        } else {
            stallTimer.stop();
            stallTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return stallTimer.hasElapsed(PivotConstants.stallDurationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setSpeed(0.0);
        startupTimer.stop();
        stallTimer.stop();
    }
}
