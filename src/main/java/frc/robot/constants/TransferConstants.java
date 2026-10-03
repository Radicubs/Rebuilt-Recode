package frc.robot.constants;

public final class TransferConstants {

    public static final int transferMotorCID = 18;

    public static final int transferMotorStallCurrentLimit = 40;
    public static final int transferMotorFreeCurrentLimit = 30;

    public static final double shootTransferSpeed = 30;
    public static final double intakeTransferSpeed = 20;

    public static final class TransferPIDFeedforwardConstants {
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.03;
        public static final double kV = 0.12;
        public static final double kA = 0.0;
    }

    
    public static final class SimConstants {
        public static final double gearing = 1.0;
        public static final double momentOfInertiaKgMetersSquared = 0.001;
    }
}