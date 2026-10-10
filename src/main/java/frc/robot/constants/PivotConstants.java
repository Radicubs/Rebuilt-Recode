package frc.robot.constants;

public final class PivotConstants {
    public static final int pivotMotorCID = 19;
    public static final int pivotMotorStallCurrentLimit = 20;
    public static final int pivotMotorFreeCurrentLimit = 10;

    public static final double downPos = 0.080555;
    public static final double upPos = -0.3662683069705963;
    public static final double middlePos = -.1;
    // Shooting folds only halfway from deployed to home, in pivot encoder rotations.
    public static final double shotFoldPosition = (downPos + upPos) / 2.0;
    public static final double foldSpeed = -0.2;
    public static final double upSpeed = -0.325;
    public static final double downSpeed = 0.325;
    public static final double stallCurrentAmps = 15.0;
    public static final double stallStartupDelaySeconds = 0.2;
    public static final double stallDurationSeconds = 0.15;
    public static final double pivotFinalVelocity = 0.0;
    public static final double maxPositionDutyCycle = 0.20;

    public static final class PIDFeedforwardConstants {
        /** Starting gain in duty cycle per pivot rotation; tune on the real mechanism. */
        public static final double P = 1.0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = 0;
        public static final double V = 0.0;
        public static final double G = 0.00;
        public static final double pidTolerance = 0.005;
    }

    // sim-only pivot model
    public static final class SimConstants {
        public static final double gearing = 60.0;
        public static final double momentOfInertiaKgMetersSquared = 0.02;
    }
}
    
