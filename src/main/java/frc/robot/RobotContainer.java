// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.pivot.MovePivotUntilStall;
import frc.robot.commands.pivot.FoldPivotWhileShooting;
import frc.robot.commands.transfer.SetTransferSpeed;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.shooter.setShooterSpeed;
import frc.robot.commands.shooter.shootOptimizedShot;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.drive.Drive;


public class RobotContainer {
    // Shooter dashboard getters need the field pose; Drive initializes VisionFunctions.
    Drive drive = Drive.getInstance();
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();
    Transfer transfer = Transfer.getInstance();
    Pivot pivot = Pivot.getInstance();

    // Ports match the USB slots in Driver Station.
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    public RobotContainer() {
        configureBindings();
        configureControllers();
    }

    private void configureControllers() {
        drive.setDefaultCommand(new TeleopDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> false
        ));
    }

    private void configureBindings() {
        operator.b().onTrue(Commands.runOnce(() -> pivot.setDefenseMode(true)));
        operator.a().onTrue(Commands.runOnce(() -> pivot.setDefenseMode(false)));

        operator.y().whileTrue(
                new setShooterSpeed(shooter, ShooterConstants.TrenchShootSpeeds.mainShooterRPS,
                        ShooterConstants.TrenchShootSpeeds.topShaftRPS,
                        ShooterConstants.TrenchShootSpeeds.indexerRPS, true)
                        .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed,
                                () -> shooter.getIndexerSetSpeed() > 0.0))
                        .alongWith(new FoldPivotWhileShooting(pivot, PivotConstants.foldSpeed)));

        operator.x().whileTrue(
                new setShooterSpeed(shooter, ShooterConstants.CloseShootSpeeds.mainShooterRPS,
                        ShooterConstants.CloseShootSpeeds.topShaftRPS,
                        ShooterConstants.CloseShootSpeeds.indexerRPS, true)
                        .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed,
                                () -> shooter.getIndexerSetSpeed() > 0.0))
                        .alongWith(new FoldPivotWhileShooting(pivot, PivotConstants.foldSpeed)));

        driver.rightTrigger().whileTrue(
                new shootOptimizedShot(shooter, transfer, pivot, drive,
                        () -> -driver.getLeftY(), () -> -driver.getLeftX()));

        driver.rightBumper().whileTrue(
                new setShooterSpeed(shooter, ShooterConstants.PassSpeeds.mainShooterRPS,
                        ShooterConstants.PassSpeeds.topShaftRPS,
                        ShooterConstants.PassSpeeds.indexerRPS, true)
                        .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed,
                                () -> shooter.getIndexerSetSpeed() > 0.0))
                        .alongWith(new FoldPivotWhileShooting(pivot, PivotConstants.foldSpeed)));

        driver.leftBumper().whileTrue(
                new SetIntakeSpeed(intake, IntakeConstants.outtakeSpeedRPS)
                        .alongWith(new SetTransferSpeed(transfer, -TransferConstants.intakeTransferSpeed)));

        driver.leftTrigger().whileTrue(
                new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS)
                        .alongWith(new SetTransferSpeed(transfer, TransferConstants.intakeTransferSpeed))
                        .alongWith(new MovePivotUntilStall(pivot, PivotConstants.downSpeed)));

        driver.x().whileTrue(Commands.runEnd(drive::lockX, drive::stop, drive));

        operator.povUp()
                .onTrue(new MovePivotUntilStall(pivot, PivotConstants.upSpeed));
        operator.povDown()
                .onTrue(new MovePivotUntilStall(pivot, PivotConstants.downSpeed));
    }

    

    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
