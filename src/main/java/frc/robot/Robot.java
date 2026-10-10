// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;



public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    
    private final RobotContainer robotContainer;
    
    
    public Robot()
    {
        Logger.recordMetadata("ProjectName", "Rebuilt-Recode");

        if (isReal())
        {
            Logger.addDataReceiver(new NT4Publisher());
        }
        else
        {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        robotContainer = new RobotContainer();
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        logCANStatus();
    }

    private void logCANStatus() {
        var status = RobotController.getCANStatus();
        double utilizationPercent = status.percentBusUtilization * 100.0;
        SmartDashboard.putNumber("CAN/UtilizationPercent", utilizationPercent);
        SmartDashboard.putNumber("CAN/BusOffCount", status.busOffCount);
        SmartDashboard.putNumber("CAN/TxFullCount", status.txFullCount);
        SmartDashboard.putNumber("CAN/ReceiveErrorCount", status.receiveErrorCount);
        SmartDashboard.putNumber("CAN/TransmitErrorCount", status.transmitErrorCount);
        Logger.recordOutput("CAN/UtilizationPercent", utilizationPercent);
        Logger.recordOutput("CAN/BusOffCount", status.busOffCount);
        Logger.recordOutput("CAN/TxFullCount", status.txFullCount);
        Logger.recordOutput("CAN/ReceiveErrorCount", status.receiveErrorCount);
        Logger.recordOutput("CAN/TransmitErrorCount", status.transmitErrorCount);
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
