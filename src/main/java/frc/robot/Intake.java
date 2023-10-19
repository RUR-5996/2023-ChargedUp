package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

    static Counter leftEnc = new Counter(new DigitalInput(0));
    static double leftTicks = 0;
    static Counter rightEnc = new Counter(new DigitalInput(1));
    static double rightTicks = 0;

    static boolean intakeIn;
    static boolean toggleIntake;
    static boolean goingOut;

    static final double LEFT_TICKS_OUT = 200;
    static final double RIGHT_TICKS_OUT = 200;

    public static void intakeInit() {
        RobotMap.leftIntake.configFactoryDefault();
        RobotMap.leftIntake.setInverted(false); //use constant
        RobotMap.leftIntake.configOpenloopRamp(0.2);
        RobotMap.leftIntake.configClosedloopRamp(0.2);
        RobotMap.leftIntake.config_kP(0, 0.5);
        RobotMap.leftIntake.config_kI(0, 0);
        RobotMap.leftIntake.config_kD(0, 0);
        RobotMap.leftIntake.setInverted(true);

        RobotMap.rightIntake.configFactoryDefault();
        RobotMap.rightIntake.setInverted(false); //use constant
        RobotMap.rightIntake.configOpenloopRamp(0.2);
        RobotMap.rightIntake.configClosedloopRamp(0.2);
        RobotMap.rightIntake.config_kP(0, 0.5);
        RobotMap.rightIntake.config_kI(0, 0);
        RobotMap.rightIntake.config_kD(0, 0);
        RobotMap.leftIntake.setInverted(false);

        intakeIn = false;
        toggleIntake = false;
    }

    public static void periodic() {
        testMode();
        intake();
    }

    public static void robotPeriodic() {
        getEncoderData();
    }

    public static void toggleIntake() {
        if(!toggleIntake) {
            RobotMap.leftIntake.set(VictorSPXControlMode.PercentOutput, 0);
            RobotMap.rightIntake.set(VictorSPXControlMode.PercentOutput, 0);
        }
        else if (intakeIn) {
            RobotMap.leftIntake.set(VictorSPXControlMode.Position, LEFT_TICKS_OUT);
            RobotMap.rightIntake.set(VictorSPXControlMode.Position, RIGHT_TICKS_OUT);        
        }
        else if (!intakeIn) {
            RobotMap.leftIntake.set(VictorSPXControlMode.Position, 0);
            RobotMap.rightIntake.set(VictorSPXControlMode.Position, 0);
        }
    }

    public static void manualIntake() {
        if(RobotMap.controller.getLeftBumper()) {
            RobotMap.leftIntake.set(VictorSPXControlMode.PercentOutput, 0.2);
            RobotMap.rightIntake.set(VictorSPXControlMode.PercentOutput, 0.2);
        } else if (RobotMap.controller.getRightBumper()) {
            RobotMap.leftIntake.set(VictorSPXControlMode.PercentOutput, -0.2);
            RobotMap.rightIntake.set(VictorSPXControlMode.PercentOutput, -0.2);
        } else {
            RobotMap.leftIntake.set(VictorSPXControlMode.PercentOutput, 0);
            RobotMap.rightIntake.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    public static void getEncoderData() {
        if(goingOut) {
            leftTicks += leftEnc.get();
            rightTicks += rightEnc.get();
        } else {
            leftTicks -= leftEnc.get();
            rightTicks -= rightEnc.get();
        }
        
        rightTicks = rightEnc.get();
        //RobotMap.leftIntake.setSelectedSensorPosition(leftTicks);
        //RobotMap.rightIntake.setSelectedSensorPosition(rightTicks);
        SmartDashboard.putNumber("left intake ticks", leftTicks);
        SmartDashboard.putNumber("right intake ticks", rightTicks);
    }

    public static void intake() {
        if(RobotMap.secondController.getRightBumper()||RobotMap.controller.getRightTriggerAxis() > 0.5) {
            RobotMap.intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.4
            
            
            );
        } else {
            RobotMap.intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    public static void testMode() {
        if(RobotMap.secondController.getLeftTriggerAxis() > 0.5) {
            RobotMap.leftIntake.set(VictorSPXControlMode.PercentOutput, 0.4);
            RobotMap.rightIntake.set(VictorSPXControlMode.PercentOutput, -0.4);
        } else if (RobotMap.secondController.getRightTriggerAxis() > 0.5) {
            RobotMap.leftIntake.set(VictorSPXControlMode.PercentOutput, -0.4);
            RobotMap.rightIntake.set(VictorSPXControlMode.PercentOutput, 0.4);
        } else {
            RobotMap.leftIntake.set(VictorSPXControlMode.PercentOutput, 0);
            RobotMap.rightIntake.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

}