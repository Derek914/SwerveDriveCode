// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  /*
   * Program entry point. This is the first code that runs when you start the
   * robot program on a PC (simulation) or on the roboRIO (real robot).
   *
   * RobotBase.startRobot(...) will construct our Robot class and hand control to
   * WPILib, which then manages the robot lifecycle (Disabled, Autonomous, Teleop,
   * Test) and calls the appropriate methods in Robot.java every 20ms.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
