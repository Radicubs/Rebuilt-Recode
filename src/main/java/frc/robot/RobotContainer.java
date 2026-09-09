// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;


public class RobotContainer{
    Intake intake = Intake.getInstance();
    private final CommandXboxController secondaryController = new CommandXboxController(1);

    public RobotContainer()
    {
        configureBindings();

    }

    
    private void configureBindings() {
        secondaryController.x().whileTrue(new SetIntakeSpeed(intake, IntakeConstants.intakeSpeedRPS));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
