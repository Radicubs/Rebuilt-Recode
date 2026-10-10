package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.transfer.Transfer;
import java.util.function.BooleanSupplier;

public class SetTransferSpeed extends Command {

    private Transfer transfer;
    private double targetSpeed;
    private final BooleanSupplier feedEnabled;
    
    public SetTransferSpeed(Transfer transfer, double targetSpeed) {
        this(transfer, targetSpeed, () -> true);
    }

    public SetTransferSpeed(Transfer transfer, double targetSpeed, BooleanSupplier feedEnabled) {
        this.transfer = transfer;
        this.targetSpeed = targetSpeed;
        this.feedEnabled = feedEnabled;
        addRequirements(transfer);
    }
    
    @Override
    public void execute() {
        if (feedEnabled.getAsBoolean()) {
            transfer.setTransferTarget(targetSpeed);
        } else {
            transfer.cancelPID();
        }
    }

    @Override
    public void end(boolean interrupted) { 
        transfer.cancelPID();
    }
}
