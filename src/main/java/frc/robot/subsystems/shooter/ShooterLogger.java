package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class ShooterLogger {
    private ShooterLogger() {}

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

        SmartDashboard.putData("Top Shooter", b -> {
            b.addDoubleProperty("Top Shooter Desired Speed", shooter::getTopDesiredSpeed, null);
        });
    }
}
