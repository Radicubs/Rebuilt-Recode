package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.vision.AutoAlignTag;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

import java.util.function.DoubleSupplier;

/** Spins up while aligning, then locks in X and feeds until released. */
public class shootOptimizedShot extends Command {

    private final Shooter shooter;
    private final Transfer transfer;
    private final Pivot pivot;
    private final Drive drive;
    private final AutoAlignTag alignment;
    private boolean reachedFoldPosition;
    private boolean wheelsLocked;

    public shootOptimizedShot(Shooter shooter, Transfer transfer, Pivot pivot, Drive drive,
        DoubleSupplier translationX, DoubleSupplier translationY) {
        this.shooter = shooter;
        this.transfer = transfer;
        this.pivot = pivot;
        this.drive = drive;
        alignment = new AutoAlignTag(drive, translationX, translationY);
        addRequirements(shooter, transfer, pivot, drive);
    }

    @Override
    public void initialize() {
        reachedFoldPosition = false;
        wheelsLocked = false;
        shooter.setOptimizedShotRPS(0.0);
        transfer.cancelPID();
        SmartDashboard.putBoolean("Wheels Locked?", false);
        foldPivot();
        alignment.initialize();
    }

    @Override
    public void execute() {
        foldPivot();
        if (!wheelsLocked) {
            alignment.execute();
            if (alignment.isAligned()) {
                alignment.end(false);
                wheelsLocked = true;
            }
        }
        if (wheelsLocked) {
            drive.lockX();
        }
        shooter.setOptimizedShotRPS(wheelsLocked ? ShooterConstants.CloseShootSpeeds.indexerRPS : 0.0);
        if (wheelsLocked) {
            transfer.setTransferTarget(TransferConstants.shootTransferSpeed);
        } else {
            transfer.cancelPID();
        }
        SmartDashboard.putBoolean("Wheels Locked?", wheelsLocked);
    }

    private void foldPivot() {
        reachedFoldPosition |= pivot.getPosition() <= PivotConstants.shotFoldPosition;
        pivot.setSpeed(reachedFoldPosition ? 0.0 : PivotConstants.foldSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        transfer.cancelPID();
        pivot.setSpeed(0.0);
        if (!wheelsLocked) {
            alignment.end(interrupted);
        } else {
            drive.stop();
        }
        wheelsLocked = false;
        SmartDashboard.putBoolean("Wheels Locked?", wheelsLocked);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
