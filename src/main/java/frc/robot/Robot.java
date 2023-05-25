// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  static SwerveDrive SWERVE;
  static double startTime = 0;

  @Override
  public void robotInit() {
    SwerveDef.flModule.moduleInit();
    SwerveDef.frModule.moduleInit();
    SwerveDef.rlModule.moduleInit();
    SwerveDef.rrModule.moduleInit();

    SWERVE = SwerveDrive.getInstance();
    Rameno.robotInit();
    Gripper.robotInit();
    Autonomous.robotInit();
    
    SWERVE.init();
    LimelightAiming.init();
  }

  @Override
  public void robotPeriodic() {
    SWERVE.report();
    //Test.report();
    Rameno.report();
    Gripper.report();
    LimelightAiming.periodic();
  }

  @Override
  public void autonomousInit() {
    Autonomous.init();
  }


  
  @Override
  public void autonomousPeriodic() {
    Autonomous.periodic();
    Rameno.autonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    SwerveDef.flModule.enabledInit(); // TODO make this cleaner
    SwerveDef.frModule.enabledInit();
    SwerveDef.rlModule.enabledInit();
    SwerveDef.rrModule.enabledInit();
    SWERVE.init();

    //Manipulator.init();
    //Test.init();
    //Test.teleopInit();

    Rameno.teleopInit();
    Gripper.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    //Manipulator.periodic();
    SWERVE.periodic();
    //Test.periodic();
    //Test.pidFollow();
    Rameno.teleopPeriodic();
    Gripper.teleopPeriodic();
  }

  @Override
  public void disabledInit() {
    SwerveDef.flModule.disabledInit(); // TODO make this cleaner
    SwerveDef.frModule.disabledInit();
    SwerveDef.rlModule.disabledInit();
    SwerveDef.rrModule.disabledInit();

    //Manipulator.disabledInit();
    //Test.disabledInit();
    Rameno.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    Rameno.disabledPeriodic();
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
