package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.transfer.Transfer;

public class SetTransferSpeed extends Command {

    private Transfer transfer;
    private double targetSpeed;
    
    public SetTransferSpeed(Transfer transfer, double targetSpeed) {
        this.transfer = transfer;
        this.targetSpeed = targetSpeed;
        addRequirements(transfer);
    }
    
    @Override
    public void execute() {
        transfer.setTransferTarget(targetSpeed);
    }

    @Override
    public void end(boolean interrupted) { 
        transfer.cancelPID();
    }
}
