package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class setShooterSpeed extends Command {

    private final Shooter shooter;
    private final DoubleSupplier mainRPS;
    private final DoubleSupplier topRPS;
    private final DoubleSupplier indexerRPS;
    private boolean waitForSpeed = false;

    public setShooterSpeed(Shooter shooter, DoubleSupplier mainRPS, DoubleSupplier topRPS, DoubleSupplier indexerRPS) {
        this.shooter = shooter;
        this.mainRPS = mainRPS;
        this.topRPS = topRPS;
        this.indexerRPS = indexerRPS;
        addRequirements(shooter);
    }

    public setShooterSpeed(Shooter shooter, double mainRPS, double topRPS, double indexerRPS) {
        this(shooter, () -> mainRPS, () -> topRPS, () -> indexerRPS);
    }

    public setShooterSpeed(Shooter shooter, double mainRPS, double topRPS, double indexerRPS,
            boolean waitForSpeed) {
        this(shooter, mainRPS, topRPS, indexerRPS);
        this.waitForSpeed = waitForSpeed;
    }

    @Override
    public void execute() {
        if (waitForSpeed) {
            shooter.setShooterRPSWhenReady(mainRPS.getAsDouble(), topRPS.getAsDouble(), indexerRPS.getAsDouble());
            return;
        }
        shooter.setShooterRPS(
            mainRPS.getAsDouble(), 
            topRPS.getAsDouble(), 
            indexerRPS.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
