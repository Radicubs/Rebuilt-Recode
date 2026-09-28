// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetIntakeSpeeds extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_subsystem = Intake.getInstance();
  private double rps;
  
  public SetIntakeSpeeds(double rps) {
    this.rps = rps;
  }

  @Override
  public void initialize() {
    m_subsystem.setVelocity(rps);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_subsystem.cancelPid();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}