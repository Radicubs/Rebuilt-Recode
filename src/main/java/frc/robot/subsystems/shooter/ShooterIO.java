package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double leftVelocityRPS = 0.0;
        public double rightVelocityRPS = 0.0;
        public double topVelocityRPS = 0.0;
        public double indexerVelocityRPS = 0.0;

        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0; 
        public double topAppliedVolts = 0.0;
        public double indexerAppliedVolts = 0.0;

        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
        public double topCurrentAmps = 0.0;
        public double indexerCurrentAmps = 0.0;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void setShooterRPS(double mainRPS, double topRPS, double indexerRPS) {}

    default void setIndexerRPS(double indexerRPS) {}

    default void stop() {}



}