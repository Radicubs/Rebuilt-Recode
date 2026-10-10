package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class ShooterLogger {
    private ShooterLogger() {}

    static void log(Shooter shooter) {
        SmartDashboard.putNumber("Shooter/MainSavedRPS", shooter.getSavedMainRPS());
        SmartDashboard.putNumber("Shooter/TopSavedRPS", shooter.getSavedTopRPS());
        SmartDashboard.putNumber("Shooter/IndexerSetRPS", shooter.getIndexerSetSpeed());
        Logger.recordOutput("Shooter/MainSavedRPS", shooter.getSavedMainRPS());
        Logger.recordOutput("Shooter/TopSavedRPS", shooter.getSavedTopRPS());
        Logger.recordOutput("Shooter/IndexerSetRPS", shooter.getIndexerSetSpeed());
        SmartDashboard.putNumber("Shooter/TopRPSAdjustment", shooter.getTopRPSAdjustment());
        SmartDashboard.putNumber("Shooter/MainRPSAdjustment", shooter.getMainRPSAdjustment());
        Logger.recordOutput("Shooter/TopRPSAdjustment", shooter.getTopRPSAdjustment());
        Logger.recordOutput("Shooter/MainRPSAdjustment", shooter.getMainRPSAdjustment());
        double topSetRPS = shooter.getTopSetSpeed();
        double mainSetRPS = shooter.getRightSetSpeed();
        SmartDashboard.putNumber("Shooter/TopSetRPS", topSetRPS);
        SmartDashboard.putNumber("Shooter/MainSetRPS", mainSetRPS);
        Logger.recordOutput("Shooter/TopSetRPS", topSetRPS);
        Logger.recordOutput("Shooter/MainSetRPS", mainSetRPS);

        double topActualRPS = shooter.getTopShooterSpeed();
        double mainActualRPS = (shooter.getLeftShooterSpeed() + shooter.getRightShooterSpeed()) / 2.0;
        SmartDashboard.putNumber("Shooter/TopActualRPS", topActualRPS);
        SmartDashboard.putNumber("Shooter/MainActualRPS", mainActualRPS);
        Logger.recordOutput("Shooter/TopActualRPS", topActualRPS);
        Logger.recordOutput("Shooter/MainActualRPS", mainActualRPS);
    }

    static void publish(Shooter shooter) {
        SmartDashboard.putData("Right Shooter", b ->
                b.addDoubleProperty("Right Shooter Speed", shooter::getRightShooterSpeed, null));

        SmartDashboard.putData("Right Shooter", b ->
                b.addDoubleProperty("Right Shooter Set Speed", shooter::getRightSetSpeed, null));

        SmartDashboard.putData("Left Shooter", b ->
                b.addDoubleProperty("Left Shooter Speed", shooter::getLeftShooterSpeed, null));

        SmartDashboard.putData("Left Shooter", b ->
                b.addDoubleProperty("Left Shooter Set Speed", shooter::getLeftSetSpeed, null));

        SmartDashboard.putData("Indexer", b -> {
            b.addDoubleProperty("Indexer Speed", shooter::getIndexerSpeed, null);
        });

        SmartDashboard.putData("Indexer", b -> {
            b.addDoubleProperty("Indexer Set Speed", shooter::getIndexerSetSpeed, null);
        });

        SmartDashboard.putData("Top Shooter", b -> {
            b.addDoubleProperty("Top Shooter Speed", shooter::getTopShooterSpeed, null);
        });

        SmartDashboard.putData("Top Shooter", b -> {
            b.addDoubleProperty("Top Shooter Set Speed", shooter::getTopSetSpeed, null);
        });

        SmartDashboard.putData("Main Shooter", b -> {
            b.addDoubleProperty("Main Shooter Desired Speed", shooter::getMainDesiredSpeed, null);
        });
    }
}
