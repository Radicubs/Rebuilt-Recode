// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetIntakeSpeeds;
import frc.robot.commands.drive.*;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.constants.IntakeConstants;

import frc.robot.subsystems.drive.Drive;


public class RobotContainer {

    CommandXboxController mainController;
    CommandXboxController secondaryController;
    // Subsystems
    // Intake intake = Intake.getInstance();
    private final SendableChooser<Command> auto_chooser = new SendableChooser<Command>();

    public RobotContainer() {
        registerNamedCommands();   // must precede the auto chooser — AutoBuilder resolves these names
        configureControllers();
        configureBindings();
    }

    private void registerNamedCommands() {
    }


    private void configureControllers() {
        mainController = new CommandXboxController(0);
        secondaryController = new CommandXboxController(1);
    }

    private void configureBindings () {

        // Main Controller Binds
        {
            mainController.rightBumper().whileTrue(new SetIntakeSpeeds(40));
        }

        // Secondary Controller Binds
        {
            // Intake (only if the pivot isn't already moving)
           

        }
    }

    public Command getAutonomousCommand () {
        return auto_chooser.getSelected();
    }
}