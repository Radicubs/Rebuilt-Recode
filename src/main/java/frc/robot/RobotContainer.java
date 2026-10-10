// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.pivot.MovePivotUntilStall;
import frc.robot.commands.pivot.SetPivotPosition;
import frc.robot.commands.pivot.ShakePivot;
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
    private final SendableChooser<Command> auto_chooser = new SendableChooser<Command>();
    public RobotContainer() {
        configureBindings();
        configureControllers();
        registerNamedCommands();
        configureAutoChooser();
    }

    private void registerNamedCommands() {
        // ---- Shooter / transfer ----
        // Ramp the flywheels (indexer held back at -3, no belt).
        NamedCommands.registerCommand("Ramp Close Shot",
                new shootOptimizedShot(shooter, transfer, pivot, drive,
                        () -> -driver.getLeftY(), () -> -driver.getLeftX()));
        NamedCommands.registerCommand("Ramp Trench Shot",
                new shootOptimizedShot(shooter, transfer, pivot, drive,
                        () -> -driver.getLeftY(), () -> -driver.getLeftX()));
        // Full shot: flywheels + indexer + belt.
        NamedCommands.registerCommand("Start Close Shot",
                new shootOptimizedShot(shooter, transfer, pivot, drive,
                        () -> -driver.getLeftY(), () -> -driver.getLeftX()));


        NamedCommands.registerCommand("Start Trench Shot",
                new shootOptimizedShot(shooter, transfer, pivot, drive,
                        () -> -driver.getLeftY(), () -> -driver.getLeftX())
                        .withTimeout(6.0));

        // Reverse/eject (shooter only, no belt).
        NamedCommands.registerCommand("Eject",
                new shootOptimizedShot(shooter, transfer, pivot, drive,
                        () -> -driver.getLeftY(), () -> -driver.getLeftX())
                        .withTimeout(1));

        // ---- Intake / pivot ----
        NamedCommands.registerCommand("Start Intake",
                new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS)
                        .alongWith(new InstantCommand(() -> pivot.setSpeed(0.07))));

        NamedCommands.registerCommand("Stop Intake", new SetIntakeSpeed(intake, 0));

        NamedCommands.registerCommand("Extend Pivot", new SetPivotPosition(pivot, PivotConstants.downPos));

        NamedCommands.registerCommand("Retract Pivot", new SetPivotPosition(pivot, PivotConstants.upPos));

        NamedCommands.registerCommand("Shake Pivot", new ShakePivot(pivot));

        NamedCommands.registerCommand("Reset Heading", new InstantCommand(() -> drive.setHeading(drive.getHeading().plus(Rotation2d.k180deg))));
    }

    private void configureControllers() {
        drive.setDefaultCommand(new TeleopDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> false
        ));
    }

    private void configureAutoChooser() {
        try {
            auto_chooser.setDefaultOption("Left Shoot", AutoBuilder.buildAuto("Left Shoot Auto"));
            auto_chooser.addOption("Middle Shoot", AutoBuilder.buildAuto("Middle Shoot Auto"));
            auto_chooser.addOption("Left Center Style Outer", AutoBuilder.buildAuto("Left Center Cycle Auto"));
            auto_chooser.addOption("Right Center Cycle", AutoBuilder.buildAuto("Right Center Cycle Auto"));
            auto_chooser.addOption("Left Center Cycle Long", AutoBuilder.buildAuto("Left Center Cycle Long Auto"));
            auto_chooser.addOption("Right Center Cycle Long", AutoBuilder.buildAuto("Right Center Cycle Long Auto"));
            auto_chooser.addOption("Middle Depot", AutoBuilder.buildAuto("Middle Depot Auto"));
        } catch (Exception e) {
            System.out.println("Error" + e.getMessage());
            auto_chooser.setDefaultOption("Auto Error", new InstantCommand());
        }
        SmartDashboard.putData("Auto Chooser", auto_chooser);
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
        return auto_chooser.getSelected();
    }
}
