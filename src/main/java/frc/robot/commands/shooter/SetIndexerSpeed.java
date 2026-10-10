package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/** Runs the indexer without changing either shooter wheel target. */
public class SetIndexerSpeed extends Command {
    private final Shooter shooter;
    private final double rps;

    public SetIndexerSpeed(Shooter shooter, double rps) {
        this.shooter = shooter;
        this.rps = rps;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setIndexerRPS(rps);
    }

    @Override
    public void execute() {
        shooter.setIndexerRPS(rps);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIndexerRPS(0.0);
    }
}
