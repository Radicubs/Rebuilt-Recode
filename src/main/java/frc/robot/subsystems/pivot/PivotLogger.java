package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class PivotLogger {
    private PivotLogger() {}

    static void publish(Pivot pivot) {
        SmartDashboard.putData("Pivot", b -> {
            b.addDoubleProperty("Desired Pivot Angle", pivot::getDesiredAngle, null);
            b.addDoubleProperty("Current Pivot Angle", pivot::getPosition, null);
        });
    }

    static void log(Pivot pivot) {
        var command = pivot.getCurrentCommand();
        String commandName = command == null ? "NONE" : command.getName();
        Logger.recordOutput("Pivot/ActiveCommand", commandName);
        SmartDashboard.putString("Pivot/ActiveCommand", commandName);
        Logger.recordOutput("Pivot/RequestedDuty", pivot.getRequestedSpeed());
        SmartDashboard.putNumber("Pivot/RequestedDuty", pivot.getRequestedSpeed());
        Logger.recordOutput("Pivot/DefenseRetracted", pivot.isDefenseRetracted());
        SmartDashboard.putBoolean("Pivot/DefenseRetracted", pivot.isDefenseRetracted());
        Logger.recordOutput("Pivot/DefenseMode", pivot.isDefenseMode());
        SmartDashboard.putBoolean("Pivot/DefenseMode", pivot.isDefenseMode());
        Logger.recordOutput("Pivot/Position", pivot.getPosition());
        Logger.recordOutput("Pivot/DesiredAngle", pivot.getDesiredAngle());
        Logger.recordOutput("Pivot/Error", pivot.getDesiredAngle() - pivot.getPosition());
        Logger.recordOutput("Pivot/AtGoal", pivot.atGoal());
        Logger.recordOutput("Pivot/MovingToTarget", pivot.isMovingToTarget());
        Logger.recordOutput("Pivot/AppliedDuty", pivot.getSpeed());
        Logger.recordOutput("Pivot/CurrentAmps", pivot.getCurrentAmps());
        SmartDashboard.putNumber("Pivot/CurrentAmps", pivot.getCurrentAmps());
        SmartDashboard.putNumber("Pivot/PositionRotations", pivot.getPosition());
        SmartDashboard.putNumber("Pivot/TargetRotations", pivot.getDesiredAngle());
        SmartDashboard.putBoolean("Pivot/MovingToTarget", pivot.isMovingToTarget());
        SmartDashboard.putNumber("Pivot/AppliedDuty", pivot.getSpeed());
    }
}
