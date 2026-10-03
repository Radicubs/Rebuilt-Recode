package frc.robot.subsystems.Transfer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DutyCycle;

interface transferIO {

    
    @AutoLog
    class transferIOInputs {
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    

    default void updateInputs(transferIOInputs inputs) {}
    default void DutyCycle(double dutyCycle) {}
    

    
}
