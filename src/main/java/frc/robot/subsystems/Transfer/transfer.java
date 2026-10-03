package frc.robot.subsystems.Transfer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.Transfer.transferIO.transferIOInputs;

public class transfer extends SubsystemBase {
    private static transfer INSTANCE;
    private final transferIOInputsAutoLogged inputs = new transferIOInputsAutoLogged();
    private final PIDController transferConroller;
    private final SimpleMotorFeedforward transferFeedforward;
    private boolean goToTransferTarget = false;
    private final transferIO io;
    

    //singleton
    public static transfer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new transfer();
        }
        return INSTANCE;
    }

    private transfer() {

        io = RobotBase.isSimulation() ? new transferIOSim() : new transferIOReal();

        transferConroller = new PIDController(
            TransferConstants.TransferPIDFeedforwardConstants.kP,
            TransferConstants.TransferPIDFeedforwardConstants.kI,
            TransferConstants.TransferPIDFeedforwardConstants.kD
        );
        transferFeedforward = new SimpleMotorFeedforward(
            TransferConstants.TransferPIDFeedforwardConstants.kS,
            TransferConstants.TransferPIDFeedforwardConstants.kV,
            TransferConstants.TransferPIDFeedforwardConstants.kA
        );
        
        TransferLogger.publish(this);
    }

    public double getTransferVelocity() {
        return inputs.velocityRPS;
    }

    
    double getSetPoint(){ 
        return transferConroller.getSetpoint();}

    boolean isActive() {
        return goToTransferTarget;
    }

    double getTransferspeed() {
        return getTransferVelocity();
    }

    public void setTransferTarget(double targetVelocity) {
        transferConroller.setSetpoint(targetVelocity);
        goToTransferTarget = true;

    }

    public void cancelPID() {
        goToTransferTarget = false;
    }
    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);
        if (goToTransferTarget) {
            double pidOutput = transferConroller.calculate(inputs.velocityRPS);
            double feedforwardOutput = transferFeedforward.calculate(transferConroller.getSetpoint());
            double totalOutput = pidOutput + feedforwardOutput;
            io.DutyCycle(totalOutput / 12.0);
            
        } 
        else {
            io.DutyCycle(0.0);
        }
    }


}
