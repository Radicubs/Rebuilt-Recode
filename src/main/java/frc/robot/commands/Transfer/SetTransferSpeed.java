package frc.robot.commands.Transfer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer.transfer;

public class SetTransferSpeed extends Command {

    private transfer transfer;
    private double targetSpeed;
    
    public SetTransferSpeed(transfer transfer, double targetSpeed) {
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
