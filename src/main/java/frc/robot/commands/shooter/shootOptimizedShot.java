package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class shootOptimizedShot extends Command {

    private final Shooter shooter;
    private final DoubleSupplier indexerRPS;

    public shootOptimizedShot(Shooter shooter, DoubleSupplier indexerRPS) {
        this.shooter = shooter;
        this.indexerRPS = indexerRPS;
        addRequirements(shooter);
    }

    public shootOptimizedShot(Shooter shooter, double indexerRPS) {
        this(shooter, () -> indexerRPS);
    }

    @Override
    public void execute() {
        shooter.setShooterRPS(
            shooter.getMainDesiredSpeed(), 
            shooter.getTopDesiredSpeed(), 
            indexerRPS.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}