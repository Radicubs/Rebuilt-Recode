package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private static Pivot INSTANCE;
    private static PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final PIDController pid;
    private final ArmFeedforward feedforward;
    private boolean moveToTarget = false; 

    public static Pivot getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Pivot();
        }
        return INSTANCE;
    }

    private Pivot() {
        io = RobotBase.isSimulation() ? new PivotIOSim() : new PivotIOReal();
        io.setPosition(PivotConstants.upPos);

        pid = new PIDController(
            PivotConstants.PIDFeedforwardConstants.P, 
            PivotConstants.PIDFeedforwardConstants.I, 
            PivotConstants.PIDFeedforwardConstants.D
        );
        pid.setTolerance(PivotConstants.PIDFeedforwardConstants.pidTolerance);

        feedforward = new ArmFeedforward(
            PivotConstants.PIDFeedforwardConstants.S, 
            PivotConstants.PIDFeedforwardConstants.V, 
            PivotConstants.PIDFeedforwardConstants.G
        );
        PivotLogger.publish(this);
    }

    public double getPosition() {
        return inputs.positionRotations;
    }
    
    public double getSpeed() {
        return inputs.appliedDuty;
    }

    public void setSpeed(double speed) {
        moveToTarget = false;
        io.setDutyCycle(speed);
    }

    public void setGoal(double targetRotation){
        moveToTarget = true;
        pid.setSetpoint(targetRotation); 
    }

    public void cancelPID(){
        io.setDutyCycle(0);
        moveToTarget = false;
        pid.reset();
    }

    public double getDesiredAngle() {
        return pid.getSetpoint();
    }

    boolean atGoal() { 
        return pid.atSetpoint(); 
    }

    boolean isMovingToTarget() { 
        return moveToTarget; 
    }

    public void resetAngle(){
        io.setPosition(PivotConstants.downPos);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        PivotLogger.log(this);

        if (moveToTarget) {
            double motorSpeed = pid.calculate(getPosition());
            
            double feedforwardVal = feedforward.calculate(
                pid.getSetpoint(), 
                0.0
            );

            io.setDutyCycle(motorSpeed + feedforwardVal);

            if (pid.atSetpoint()) {
                cancelPID();
            }
        }
    }
}