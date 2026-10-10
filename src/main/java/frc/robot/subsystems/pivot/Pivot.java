package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private static Pivot INSTANCE;
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final PIDController pid;
    private final ArmFeedforward feedforward;
    private boolean moveToTarget = false; 
    private boolean defenseMode = false;
    private boolean defenseRetracted = false;
    private final Timer defenseStartupTimer = new Timer();
    private final Timer defenseStallTimer = new Timer();

    public static Pivot getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Pivot();
        }
        return INSTANCE;
    }

    private Pivot() {
        this(RobotBase.isSimulation() ? new PivotIOSim() : new PivotIOReal());
        PivotLogger.publish(this);
    }

    Pivot(PivotIO io) {
        this.io = io;
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
    }

    /** Retract to the end stop and retain brake until normal mode is selected. */
    public void setDefenseMode(boolean enabled) {
        defenseMode = enabled;
        defenseRetracted = false;
        moveToTarget = false;
        pid.reset();
        defenseStallTimer.stop();
        defenseStallTimer.reset();
        defenseStartupTimer.stop();
        defenseStartupTimer.reset();
        if (enabled) {
            defenseStartupTimer.start();
        }
        io.setDutyCycle(enabled ? PivotConstants.upSpeed : 0.0);
    }

    public boolean isDefenseMode() {
        return defenseMode;
    }

    public double getPosition() {
        return inputs.positionRotations;
    }
    
    public double getSpeed() {
        return inputs.appliedDuty;
    }

    public double getRequestedSpeed() {
        return inputs.requestedDuty;
    }

    boolean isDefenseRetracted() {
        return defenseRetracted;
    }

    public double getCurrentAmps() {
        return inputs.currentAmps;
    }

    public void setSpeed(double speed) {
        if (defenseMode) { return; }
        moveToTarget = false;
        io.setDutyCycle(speed);
    }

    public void setGoal(double targetRotation){
        if (defenseMode) { return; }
        moveToTarget = true;
        pid.setSetpoint(targetRotation); 
    }

    public void cancelPID(){
        if (defenseMode) { return; }
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

        if (defenseMode) {
            if (!defenseRetracted) {
                if (defenseStartupTimer.hasElapsed(PivotConstants.stallStartupDelaySeconds)
                        && getCurrentAmps() >= PivotConstants.stallCurrentAmps) {
                    defenseStallTimer.start();
                } else {
                    defenseStallTimer.stop();
                    defenseStallTimer.reset();
                }
                defenseRetracted = defenseStallTimer.hasElapsed(PivotConstants.stallDurationSeconds);
            }
            // PivotIOReal uses brake idle mode, so zero output brakes after retraction.
            io.setDutyCycle(defenseRetracted ? 0.0 : PivotConstants.upSpeed);
        } else if (moveToTarget) {
            double motorSpeed = pid.calculate(getPosition());
            
            double feedforwardVal = feedforward.calculate(
                pid.getSetpoint(), 
                0.0
            );

            io.setDutyCycle(MathUtil.clamp(motorSpeed + feedforwardVal,
                    -PivotConstants.maxPositionDutyCycle, PivotConstants.maxPositionDutyCycle));

            if (pid.atSetpoint()) {
                cancelPID();
            }
        }
        PivotLogger.log(this);
    }
}
