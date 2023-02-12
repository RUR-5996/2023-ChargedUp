// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  static SwerveDrive SWERVE;

  @Override
  public void robotInit() {
    SwerveDef.flModule.moduleInit();
    SwerveDef.frModule.moduleInit();
    SwerveDef.rlModule.moduleInit();
    SwerveDef.rrModule.moduleInit();

    SWERVE = SwerveDrive.getInstance();
  }

  @Override
  public void robotPeriodic() {
    SWERVE.report();
  }

  @Override
  public void autonomousInit() {
    Autonomous.init();
  }

  @Override
  public void autonomousPeriodic() {
    Autonomous.periodic();
  }

  @Override
  public void teleopInit() {
    SwerveDef.flModule.enabledInit(); //TODO make this cleaner
    SwerveDef.frModule.enabledInit();
    SwerveDef.rlModule.enabledInit();
    SwerveDef.rrModule.enabledInit();
    SWERVE.init();
  }

  @Override
  public void teleopPeriodic() {
    SwerveDrive.periodic();
    //SwerveDrive.test();
  }

  @Override
  public void disabledInit() {
    SwerveDef.flModule.disabledInit(); //TODO make this cleaner
    SwerveDef.frModule.disabledInit();
    SwerveDef.rlModule.disabledInit();
    SwerveDef.rrModule.disabledInit();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
