package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final PIDController controller;
    private boolean PIDActive = false;
    private final SimpleMotorFeedforward feedforward;
    

    
    public static Intake getInstance() {
        if (INSTANCE == null) {INSTANCE = new Intake();}
        return INSTANCE;
    }

    private Intake() {
        io = RobotBase.isSimulation() ? new IntakeIOSim() : new IntakeIOReal();
        io.updateInputs(inputs);
        IntakeLogger.publish(this);
        controller = new PIDController(IntakeConstants.PIDFeedforwardConstants.P, IntakeConstants.PIDFeedforwardConstants.I, IntakeConstants.PIDFeedforwardConstants.D);
        feedforward = new SimpleMotorFeedforward(IntakeConstants.PIDFeedforwardConstants.S, IntakeConstants.PIDFeedforwardConstants.V, IntakeConstants.PIDFeedforwardConstants.A);
        controller.setTolerance(IntakeConstants.PIDFeedforwardConstants.pidTolerance);
           
    }

    public void setTransferSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
        PIDActive = true;
    }

    public double getVelocity() {
        return inputs.velocityRPS;
    }

    public double getAppliedOutput() {
        return inputs.appliedVolts;
        
    }


    public double setVelocity(double velocityRPS) {
        controller.reset();
        PIDActive = true;
        controller.setSetpoint(velocityRPS);
        return velocityRPS; 
    }

    public double getSetpoint() {
        return controller.getSetpoint();
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public void cancelPID() {
        PIDActive = false;
        io.setDutyCycle(0);
    }

    public boolean isPidEnabled() {
        return PIDActive;
    }

    public  double getIntakeSpeedRPS() {
        return IntakeConstants.intakeSpeedRPS;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        if (PIDActive) {
            double pidOutput = controller.calculate(inputs.velocityRPS);
            double feedforwardOutput = feedforward.calculate(controller.getSetpoint());
            io.setDutyCycle(pidOutput + feedforwardOutput);
        }

        
        IntakeLogger.log(this);
    }
}
