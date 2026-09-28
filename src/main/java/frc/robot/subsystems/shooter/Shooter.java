package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private static Shooter INSTANCE;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean pidEnabled = false;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;

    public static Shooter getInstance() {
        if (INSTANCE == null) {INSTANCE = new Shooter();}
        return INSTANCE;
    }

    private Shooter() {
        io = RobotBase.isSimulation() ? new IntakeIOSim() : new ShooterIOReal();

        controller = new PIDController(IntakeConstants.PIDFeedforwardConstants.P, IntakeConstants.PIDFeedforwardConstants.I, IntakeConstants.PIDFeedforwardConstants.D);
        controller.setTolerance(IntakeConstants.PIDFeedforwardConstants.pidTolerance);

        feedforward = new SimpleMotorFeedforward(IntakeConstants.PIDFeedforwardConstants.S, IntakeConstants.PIDFeedforwardConstants.V, IntakeConstants.PIDFeedforwardConstants.A);

        io.updateInputs(inputs);
        ShooterLogger.publish(this);
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

    public void setVelocity(double rps){
        controller.reset();
        controller.setSetpoint(rps);
        pidEnabled = true;
    }

    public boolean isPidEnabled(){
        return pidEnabled;
    }

    public void cancelPid(){
        controller.setSetpoint(0);
        pidEnabled = false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (pidEnabled) {
            double output = controller.calculate(inputs.velocityRPS)
                    + feedforward.calculate(controller.getSetpoint());
            io.setDutyCycle(output);
        }

        ShooterLogger.log(this);
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    }
}
