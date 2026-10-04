package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private static Transfer INSTANCE;
    private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();
    private final PIDController transferConroller;
    private final SimpleMotorFeedforward transferFeedforward;
    private boolean goToTransferTarget = false;
    private final TransferIO io;
    

    //singleton
    public static Transfer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Transfer();
        }
        return INSTANCE;
    }

    private Transfer() {

        io = RobotBase.isSimulation() ? new TransferIOSim() : new TransferIOReal();

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
        transferConroller.reset();
        io.DutyCycle(0.0);
    }
    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);
        if (goToTransferTarget) {
            double pidOutput = transferConroller.calculate(inputs.velocityRPS);
            double feedforwardOutput = transferFeedforward.calculate(transferConroller.getSetpoint());
            double totalOutput = pidOutput + feedforwardOutput;
            io.DutyCycle(MathUtil.clamp(totalOutput / 12.0, -1.0, 1.0));
            
        } 
        else {
            io.DutyCycle(0.0);
        }
    }


}
