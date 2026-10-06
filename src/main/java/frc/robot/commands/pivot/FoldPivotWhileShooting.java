package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.constants.PivotConstants;

public class FoldPivotWhileShooting extends Command {

   private static final double FOLD_SPEED_RATE = 0.05; 

    private final Pivot pivot;
    private final double degrees;
    private final Timer timer = new Timer();

    public FoldPivotWhileShooting(Pivot pivot, double degrees) {
        this.pivot = pivot;
        this.degrees = degrees;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double elapsedTime = timer.get();
        
        double targetPosition = degrees - (elapsedTime * FOLD_SPEED_RATE);
        
        double minLimit = Math.min(PivotConstants.upPos, PivotConstants.downPos);
        double maxLimit = Math.max(PivotConstants.upPos, PivotConstants.downPos);
        targetPosition = Math.max(minLimit, Math.min(maxLimit, targetPosition));

        pivot.setGoal(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setGoal(PivotConstants.downPos);
    }
}