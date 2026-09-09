package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import pabeles.concurrency.IntRangeTask;

import java.util.function.DoubleSupplier;

import javax.annotation.processing.SupportedOptions;

public class SetIntakeSpeed extends Command {
    private final Intake intake;
    private final DoubleSupplier intakeRPS;

    public SetIntakeSpeed(Intake intake, DoubleSupplier intakeRPS) {
        this.intake = intake;
        this.intakeRPS = intakeRPS;
        addRequirements(intake);
        
    }

    public SetIntakeSpeed(Intake intake, double intakeRPS) {
        this(intake, () -> intakeRPS);  
    }

    @Override
    public void execute() {
        intake.setVelocity(intakeRPS.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.cancelPID();

     }
    }

