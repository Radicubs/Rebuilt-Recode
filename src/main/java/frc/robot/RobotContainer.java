// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.Transfer.SetTransferSpeed;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.shooter.setShooterSpeed;
import frc.robot.commands.vision.AutoAlignTag;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Transfer.transfer;
import frc.robot.subsystems.drive.Drive;



public class RobotContainer{
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();
    transfer transfer = frc.robot.subsystems.Transfer.transfer.getInstance();
    Drive drive = Drive.getInstance();


    private CommandXboxController operator = new CommandXboxController(1);
        private CommandXboxController driver = new CommandXboxController(0);
        public RobotContainer()
        {
            configureBindings();
            configureControllers();
        }
    
        
        private void configureControllers() {
            driver = new CommandXboxController(0);
            operator = new CommandXboxController(1);

        drive.setDefaultCommand(new TeleopDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> false //mainController.x().getAsBoolean()
        ));
    }
    private void configureBindings() {
        operator.x().whileTrue(new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS));
    
        operator.rightBumper().whileTrue(
            new setShooterSpeed(shooter,ShooterConstants.CloseShootSpeeds.mainShooterRPS , ShooterConstants.CloseShootSpeeds.topShaftRPS, ShooterConstants.CloseShootSpeeds.indexerRPS)
                .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed))
        );

        driver.rightTrigger().whileTrue(new AutoAlignTag(
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
