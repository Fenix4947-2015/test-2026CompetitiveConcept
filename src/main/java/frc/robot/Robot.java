// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot  {
    private final RobotContainer m_robotContainer;
    private Command m_autonomousCommand;
    private final Field2d m_field = new Field2d();
    private long lastVisionUpdateTime = 0;  // Timestamp of the last vision telemetry update

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData("Field", m_field);
        RobotController.setBrownoutVoltage(Volts.of(6.1));
        lastVisionUpdateTime = System.currentTimeMillis();
    }
    
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // throttle the telemetry updates to avoid flooding the network tables and overwhelming the dashboard
        if (System.currentTimeMillis() - lastVisionUpdateTime > 200) {
            // Call the RobotContainer's periodic method to update vision telemetry
            m_robotContainer.Periodic();  

            // Get the latest robot pose from odometry and update the Field2d for visualization on the dashboard
            m_field.setRobotPose(m_robotContainer.getPoseMeters());

            lastVisionUpdateTime = System.currentTimeMillis();
        }
    }

    @Override
    public void robotInit() {

        Logger.addDataReceiver(new WPILOGWriter());   // writes .wpilog file
        Logger.addDataReceiver(new NT4Publisher());   // streams live to NT
        Logger.start();
    }

    @Override
    public void autonomousInit() {
        // Retrieve the selected autonomous command from the RobotContainer and schedule it
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        // Ensure any running autonomous command is cancelled when teleop starts
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
            m_autonomousCommand = null;
        }
    }

    @Override
    public void disabledInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
            m_autonomousCommand = null;
        }
    }
}
