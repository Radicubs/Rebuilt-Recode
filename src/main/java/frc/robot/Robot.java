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
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.constants.DriveConstants;

import java.io.File;



public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    
    private final RobotContainer robotContainer;

    private int canLogCounter = 0;
    
    
    public Robot()
    {
        Logger.recordMetadata("ProjectName", "Rebuilt-Recode");

        if (isReal())
        {
            Logger.addDataReceiver(new NT4Publisher());
            // Match logs go to a USB stick when one is plugged in (keeps roboRIO storage free).
            if (new File("/U").isDirectory())
            {
                Logger.addDataReceiver(new WPILOGWriter());
            }
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
        // CAN health at 5 Hz: enough for pit checks without loading the main loop.
        if (++canLogCounter >= 10)
        {
            canLogCounter = 0;
            logCANStatus();
        }
    }

    private void logCANStatus()
    {
        var rio = RobotController.getCANStatus();
        SmartDashboard.putNumber("CAN/RIO/UtilizationPercent", rio.percentBusUtilization * 100.0);
        SmartDashboard.putNumber("CAN/RIO/BusOffCount", rio.busOffCount);
        SmartDashboard.putNumber("CAN/RIO/TxFullCount", rio.txFullCount);
        Logger.recordOutput("CAN/RIO/UtilizationPercent", rio.percentBusUtilization * 100.0);
        Logger.recordOutput("CAN/RIO/BusOffCount", rio.busOffCount);
        Logger.recordOutput("CAN/RIO/TxFullCount", rio.txFullCount);

        if (isReal())
        {
            var canivore = DriveConstants.canBus.getStatus();
            boolean ok = canivore.Status.isOK();
            SmartDashboard.putBoolean("CAN/CANivore/OK", ok);
            SmartDashboard.putNumber("CAN/CANivore/UtilizationPercent", canivore.BusUtilization * 100.0);
            SmartDashboard.putNumber("CAN/CANivore/BusOffCount", canivore.BusOffCount);
            SmartDashboard.putNumber("CAN/CANivore/TxFullCount", canivore.TxFullCount);
            Logger.recordOutput("CAN/CANivore/OK", ok);
            Logger.recordOutput("CAN/CANivore/UtilizationPercent", canivore.BusUtilization * 100.0);
            Logger.recordOutput("CAN/CANivore/BusOffCount", canivore.BusOffCount);
            Logger.recordOutput("CAN/CANivore/TxFullCount", canivore.TxFullCount);
        }
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
