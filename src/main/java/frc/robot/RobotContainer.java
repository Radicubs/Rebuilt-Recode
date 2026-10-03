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


    private CommandXboxController secondaryController = new CommandXboxController(1);
        private CommandXboxController mainController = new CommandXboxController(0);
        public RobotContainer()
        {
            configureBindings();
            configureControllers();
        }
    
        
        private void configureControllers() {
            mainController = new CommandXboxController(0);
            secondaryController = new CommandXboxController(1);

        drive.setDefaultCommand(new TeleopDrive(
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> false //mainController.x().getAsBoolean()
        ));
    }
    private void configureBindings() {
        secondaryController.x().whileTrue(new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS));
    
        secondaryController.rightBumper().whileTrue(
            new setShooterSpeed(shooter,ShooterConstants.CloseShootSpeeds.mainShooterRPS , ShooterConstants.CloseShootSpeeds.topShaftRPS, ShooterConstants.CloseShootSpeeds.indexerRPS)
                .alongWith(new SetTransferSpeed(transfer, TransferConstants.shootTransferSpeed))
        );
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
