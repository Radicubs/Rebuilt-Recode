package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.TransferConstants;

import org.littletonrobotics.junction.Logger;

final class TransferLogger {

    private TransferLogger() {}

    static void publish(Transfer transfer) {
        SmartDashboard.putData("Transfer", b ->
                b.addDoubleProperty("Transfer Speed", transfer::getTransferVelocity, null));
    }

    static void log(Transfer transfer) {
        Logger.recordOutput("Transfer/Setpoint", transfer.getSetPoint());
        Logger.recordOutput("Transfer/Error", transfer.getSetPoint() - transfer.getTransferVelocity());
        Logger.recordOutput("Transfer/Active", transfer.isActive());
        Logger.recordOutput("Transfer/Velocity", transfer.getTransferVelocity());
        Logger.recordOutput("Transfer/P", TransferConstants.TransferPIDFeedforwardConstants.kP);
        Logger.recordOutput("Transfer/I", TransferConstants.TransferPIDFeedforwardConstants.kI);
        Logger.recordOutput("Transfer/D", TransferConstants.TransferPIDFeedforwardConstants.kD);
        
     
    }

    
}