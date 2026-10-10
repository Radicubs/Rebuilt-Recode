package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    class PivotIOInputs {
        public double positionRotations = 0.0;
        public double currentAmps = 0.0;
        public double appliedDuty = 0.0;
        public double requestedDuty = 0.0;
        public double appliedVolts = 0.0;
    }

    default void updateInputs(PivotIOInputs inputs) {}
    default void setDutyCycle(double DutyCycle) {}
    default void setPosition(double positionRotations) {}
}
