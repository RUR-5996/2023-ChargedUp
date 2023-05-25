package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gripper {

    static boolean coneMode = false;
    static boolean onlyIntake = false;
    static double coneSpeed = -0.9;
    static double cubeSpeed = 0.9;
    
    public static void motorInit() {
        RobotMap.gripper.configFactoryDefault();
        RobotMap.gripper.configOpenloopRamp(0.2);
        RobotMap.gripper.configClosedloopRamp(0.2);
        RobotMap.gripper.setNeutralMode(NeutralMode.Brake);
    }

    public static void robotInit() {
        motorInit();
    }

    public static void teleopInit() {

    }

    public static void switchMode() { //TODO change button layout
        if(RobotMap.secondController.getPOV() == 90) {
            coneMode = false; // useless
        } /*else if(RobotMap.secondController.getPOV() == 270) {
            coneMode = false;
        }*/
    }

    public static void intake() {
        if(coneMode) {
            RobotMap.gripper.set(coneSpeed);
        } else if(!coneMode) {
            RobotMap.gripper.set(cubeSpeed);
        } else {
            RobotMap.gripper.set(0); //unlikely, should handle some weird errors
        }
    }

    public static void outtake() {
        if(coneMode && !onlyIntake) {
            RobotMap.gripper.set(-coneSpeed);
        } else if (!coneMode && !onlyIntake) {
            RobotMap.gripper.set(-cubeSpeed);
        } else {
            RobotMap.gripper.set(0);
        }
    }

    public static void gripAutonomous(boolean isIntake, boolean isCone){
        double finalSpeed = isCone ? coneSpeed : cubeSpeed;
        if(isIntake){
            RobotMap.gripper.set(finalSpeed);
        }
        else{
            RobotMap.gripper.set(-finalSpeed);
        }
    }

    public static void disableMotor() {
        RobotMap.gripper.set(0);
    }

    public static void teleopPeriodic() {
        if(RobotMap.secondController.getLeftTriggerAxis() > 0.5 || RobotMap.secondController.getRightBumper() || RobotMap.controller.getLeftTriggerAxis() > 0.5 || RobotMap.controller.getRightBumper()) {
            outtake();
        } else if (RobotMap.secondController.getRightTriggerAxis() > 0.5 || RobotMap.secondController.getLeftBumper() || RobotMap.controller.getRightTriggerAxis() > 0.5 || RobotMap.controller.getLeftBumper()) {
            intake();
        } else {
            disableMotor();
        }

        switchMode();
    }

    public static void report() {
        SmartDashboard.putNumber("motor voltage", RobotMap.gripper.getMotorOutputVoltage()); //the only option of monitoring motor load - for testing purposes
        SmartDashboard.putBoolean("cone mode", coneMode);
    }

    public static void limitIntake() {
        if (Rameno.currentPositionId == 1) {
            onlyIntake = true;
        }
    }
}
