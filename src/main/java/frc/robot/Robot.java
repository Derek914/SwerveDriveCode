// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // Build and wire up the entire robot here. RobotContainer sets up
    // subsystems (like the swerve drivetrain), default behavior, and controller bindings.
    m_robotContainer = new RobotContainer();
  }

  /**
   * Runs every 20ms in all modes (Disabled, Autonomous, Teleop, Test, Simulation).
   * We call the CommandScheduler here so default commands run and button bindings
   * are processed. This is the "heartbeat" that keeps the robot responsive.
   */
  @Override
  public void robotPeriodic() { 
    CommandScheduler.getInstance().run(); 
  }

  /** Called once when the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  /** Runs every 20ms while the robot is Disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Called once when the robot leaves Disabled mode. */
  @Override
  public void disabledExit() {}

  /**
   * Called once when Autonomous mode starts. We get the autonomous command
   * from RobotContainer and schedule it to start running.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** Runs every 20ms during Autonomous mode. */
  @Override
  public void autonomousPeriodic() {}

  /** Called once when Autonomous mode ends. */
  @Override
  public void autonomousExit() {}

  /**
   * Called once when Teleop mode starts. If an autonomous command is still
   * running, cancel it so driver control takes over cleanly.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Runs every 20ms during Teleop mode. */
  @Override
  public void teleopPeriodic() {}

  /** Called once when Teleop mode ends. */
  @Override
  public void teleopExit() {}

  /**
   * Called once when Test mode starts. Cancel all running commands so you
   * can test subsystems safely and in isolation.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Runs every 20ms during Test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when Test mode ends. */
  @Override
  public void testExit() {}

  /** Runs every 20ms during simulation. */
  @Override
  public void simulationPeriodic() {}
}
