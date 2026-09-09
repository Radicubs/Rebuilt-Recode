package frc.robot.subsystems.Intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeIOInputsAutoLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean pidEnabled = false;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;

    public static Intake getInstance() {
        if (INSTANCE == null) {INSTANCE = new Intake();}
        return INSTANCE;
    }

    private Intake() {
        io = RobotBase.isSimulation() ? new IntakeIOSim() : new IntakeIOReal();

        controller = new PIDController(IntakeConstants.PIDFeedforwardConstants.P, IntakeConstants.PIDFeedforwardConstants.I, IntakeConstants.PIDFeedforwardConstants.D);
        controller.setTolerance(IntakeConstants.PIDFeedforwardConstants.pidTolerance);

        feedforward = new SimpleMotorFeedforward(IntakeConstants.PIDFeedforwardConstants.S, IntakeConstants.PIDFeedforwardConstants.V, IntakeConstants.PIDFeedforwardConstants.A);

        io.updateInputs(inputs);
        IntakeLogger.publish(this);
    }


    public double getVelocity() {
        return inputs.velocityRPS;
    }

    public double getSetpoint(){
        return controller.getSetpoint();
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public double setVelocity(double velocityRPS) {
        controller.reset();
        pidEnabled = true;
        controller.setSetpoint(velocityRPS);
        return velocityRPS;
    }

    public void cancelPID() {
        pidEnabled = false;
        io.setDutyCycle(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (pidEnabled) {
            double output = controller.calculate(inputs.velocityRPS)
                    + feedforward.calculate(controller.getSetpoint());
            io.setDutyCycle(output);
        }

        IntakeLogger.log(this);
    }
}
