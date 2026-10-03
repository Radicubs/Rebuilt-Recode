package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IntakeConstants;

import org.littletonrobotics.junction.Logger;

final class IntakeLogger {

    private IntakeLogger() {}

    static void publish(Intake intake) {
        SmartDashboard.putData("Intake", b ->
                b.addDoubleProperty("Intake Speed", intake::getVelocity, null));
    }

    static void log(Intake intake) {
        Logger.recordOutput("Intake/Setpoint", intake.getSetpoint());
        Logger.recordOutput("Intake/Error", intake.getSetpoint() - intake.getVelocity());
        Logger.recordOutput("Intake/AtSetpoint", intake.atSetpoint());
        Logger.recordOutput("Intake/PidEnabled", intake.isPidEnabled());

        Logger.recordOutput("Intake/Velocity", intake.getVelocity());
        Logger.recordOutput("Intake/P", IntakeConstants.PIDFeedforwardConstants.P);
        Logger.recordOutput("Intake/I", IntakeConstants.PIDFeedforwardConstants.I);
        Logger.recordOutput("Intake/D", IntakeConstants.PIDFeedforwardConstants.D);
        Logger.recordOutput("Intake/S", IntakeConstants.PIDFeedforwardConstants.S);
        Logger.recordOutput("Intake/V", IntakeConstants.PIDFeedforwardConstants.V);
        Logger.recordOutput("Intake/A", IntakeConstants.PIDFeedforwardConstants.A);
        Logger.recordOutput("Intake/G", IntakeConstants.PIDFeedforwardConstants.G);
        Logger.recordOutput("Intake/IntakeSpeedRPS", Intake.getInstance().getIntakeSpeedRPS());
        Logger.recordOutput("Intake/Voltage", Intake.getInstance().getAppliedOutput());

    }
}