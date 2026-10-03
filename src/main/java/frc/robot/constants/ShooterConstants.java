package frc.robot.constants;

public final class ShooterConstants
{
    public static final int rightShooterCID = 15;
    public static final int leftShooterCID = 16;

    public static final double pidTolerance = 0.5;

    public static final int indexerCID = 14;
    public static final int topShooterCID = 13;

    public static final int topShooterCurrentLimit = 35;
    public static final boolean topShooterEnableCurrentLimit = true;

    public static final int indexerShooterCurrentLimit = 35;
    public static final boolean indexerEnableCurrentLimit = true;

    public static final int shooterCurrentLimit = 35;
    public static final boolean shooterEnableCurrentLimit = true;

    public static final class CloseShootSpeeds{
        public static double mainShooterRPS = 100; 
        public static final double topShaftRPS = 40; 
        public static final double indexerRPS = 20; 
    }
    public static final class TrenchShootSpeeds{
        public static double mainShooterRPS = 60; 
        public static final double topShaftRPS = 10; 
        public static final double indexerRPS = 20; 
    }
    public static final class PassSpeeds{
        public static double mainShooterRPS = 30; 
        public static final double topShaftRPS = 20; 
        public static final double indexerRPS = 20; 
    }

    public static final class EjectSpeeds{
        public static double mainShooterRPS = -10;
        public static double topShaftRPS = -10;
        public static double indexerRPS = -10;
    }


    public static final class MainRightShooterPIDFeedforwardConstants {
        public static final double kP = 0.3; // 0.3
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kS = 0.12;
        public static final double kV = 0.11    ;//.11
        public static final double kA = 0;
    }

    public static final class MainLeftShooterPIDFeedforwardConstants {
        public static final double kP = 0.3; //0.3
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.12;
        public static final double kV = 0.11  ;//.11;
        public static final double kA = 0.0;
    } 

    public static final class IndexerPIDFeedforwardConstants {
        public static final double kP = 0.02; //.13
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = .12;
        public static final double kV = .12;
        public static final double kA = 0.0;
    }

    public static final class TopShooterPIDFeedforwardConstants {
        public static final double kP = 0.22;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.16;
        public static final double kV = 0.115;
        public static final double kA = 0.0;
    }


    public static final class SimConstants {
        public static final double mainShooterMoiKgM2 = 0.01;   
        public static final double topShooterMoiKgM2 = 0.003;
        public static final double indexerMoiKgM2 = 0.001;
    }
}