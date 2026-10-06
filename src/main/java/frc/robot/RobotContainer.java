// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.pivot.FoldPivotWhileShooting;
import frc.robot.commands.pivot.SetPivotPosition;
import frc.robot.commands.pivot.ShakePivot;
import frc.robot.commands.transfer.SetTransferSpeed;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.shooter.setShooterSpeed;
import frc.robot.commands.shooter.shootOptimizedShot;
import frc.robot.commands.vision.AutoAlignTag;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;



public class RobotContainer{
    // Shooter dashboard getters need the field pose; Drive initializes VisionFunctions.
    Drive drive = Drive.getInstance();
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();
    Transfer transfer = Transfer.getInstance();
    Pivot pivot = Pivot.getInstance();


    private CommandXboxController operator = new CommandXboxController(0);
        // private CommandXboxController driver = new CommandXboxController(0);
        private CommandPS5Controller driver = new CommandPS5Controller(1);
        public RobotContainer()
        {
            configureBindings();
            configureControllers();
        }
    
        
        private void configureControllers() {
            // driver =  new CommandPS5Controller(0);
            // operator = new CommandXboxController(1);

        drive.setDefaultCommand(new TeleopDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> false //mainController.x().getAsBoolean()
        ));
    }
    private void configureBindings() {
        operator.x().whileTrue(new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS).alongWith(new SetTransferSpeed(transfer, TransferConstants.intakeTransferSpeed)));
    
        operator.rightBumper().whileTrue(new setShooterSpeed(shooter,ShooterConstants.CloseShootSpeeds.mainShooterRPS , ShooterConstants.CloseShootSpeeds.topShaftRPS, ShooterConstants.CloseShootSpeeds.indexerRPS).alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed)).alongWith(new FoldPivotWhileShooting(pivot, PivotConstants.middlePos)));

        operator.rightTrigger().whileTrue(new setShooterSpeed(shooter, ShooterConstants.TrenchShootSpeeds.mainShooterRPS, ShooterConstants.TrenchShootSpeeds.topShaftRPS, ShooterConstants.TrenchShootSpeeds.indexerRPS).alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed)).alongWith(new FoldPivotWhileShooting(pivot, 0 )));

        operator.leftTrigger().whileTrue(new setShooterSpeed(shooter, ShooterConstants.PassSpeeds.mainShooterRPS, ShooterConstants.PassSpeeds.topShaftRPS, ShooterConstants.PassSpeeds.indexerRPS).alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed)).alongWith(new FoldPivotWhileShooting(pivot, .05)));

        operator.leftBumper().whileTrue(new setShooterSpeed(shooter, ShooterConstants.EjectSpeeds.mainShooterRPS, ShooterConstants.EjectSpeeds.topShaftRPS, ShooterConstants.EjectSpeeds.indexerRPS).alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed)));

        operator.povUp().onTrue(new SetPivotPosition(pivot, PivotConstants.upPos));


        operator.rightTrigger().whileTrue(new shootOptimizedShot(shooter, 20)
            .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed))
        );


        // driver.rightTrigger().whileTrue(new AutoAlignTag(
        //     drive,
        //     () -> -driver.getLeftY(),
        //     () -> -driver.getLeftX()
        // ));
        driver.R2().whileTrue(new AutoAlignTag(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX()
        ));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
