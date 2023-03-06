package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test {
    
    static double FALCON_TICKS_PER_REV = 2048.0;
    static double ARM_GEAR_RATIO = 0.009375;

    static double coeff = 3;
    static double speed1 = 0.19;
    static double speed2 = 0.18;
    static double antiGravity = 0;
    static TalonFXInvertType mover1Inverted = TalonFXInvertType.Clockwise;
    static TalonFXInvertType mover2Inverted = TalonFXInvertType.CounterClockwise;
    static double feedbackCoefficient = FALCON_TICKS_PER_REV * ARM_GEAR_RATIO * 360; //counts in arm degrees TODO larger scale
    //static boolean mover1Inverted = false;
    //static boolean mover2Inverted = false;
    static double testPosition = 360;
    static boolean sensing = false;
    static Timer releaseTimer = new Timer();

    public static void init() {
        RobotMap.mover1.configFactoryDefault();
        RobotMap.mover1.setInverted(mover1Inverted);
        RobotMap.mover1.configOpenloopRamp(0);
        RobotMap.mover1.configClosedloopRamp(0); //try this to save up the gears
        RobotMap.mover1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        RobotMap.mover1.setSelectedSensorPosition(0);
        RobotMap.mover1.configSelectedFeedbackCoefficient(feedbackCoefficient);
        RobotMap.mover1.config_kP(0, 1);
        RobotMap.mover1.config_kI(0, 0);
        RobotMap.mover1.config_kD(0, 0);

        RobotMap.mover2.configFactoryDefault();
        RobotMap.mover2.setInverted(mover2Inverted);
        RobotMap.mover2.configOpenloopRamp(0);
        RobotMap.mover2.configClosedloopRamp(0);
        RobotMap.mover2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        RobotMap.mover2.setSelectedSensorPosition(0);
        RobotMap.mover2.config_kP(0, 0.3);
        RobotMap.mover2.config_kI(0, 0);
        RobotMap.mover2.config_kD(0, 0);
        RobotMap.mover2.follow(RobotMap.mover1);

        RobotMap.release.configFactoryDefault();
        RobotMap.release.setInverted(false); //use constant
        RobotMap.release.configOpenloopRamp(0.2);
        RobotMap.release.configClosedloopRamp(0.2);

        //RobotMap.mover2.follow(RobotMap.mover1);
    }

    public static void teleopInit() {

        RobotMap.mover1.setNeutralMode(NeutralMode.Brake);
        RobotMap.mover2.setNeutralMode(NeutralMode.Brake);

    }

    public static void startRelease() {
        releaseTimer.reset();
    }

    public static void disabledInit() {
        RobotMap.mover1.setNeutralMode(NeutralMode.Coast);
        RobotMap.mover2.setNeutralMode(NeutralMode.Coast);
    }

    public static void periodic() { //TODO clean this up probably with enums and better constant system
        if(sensing) {
            if(-RobotMap.secondController.getLeftY() > 0.5) {
                RobotMap.mover1.set(ControlMode.PercentOutput, speed1*coeff);
                //RobotMap.mover2.set(ControlMode.PercentOutput, speed2*coeff);
            } else {
                RobotMap.mover1.set(ControlMode.PercentOutput, speed1*antiGravity); //has to be low, could be compensated for by the pid loop if running constantly
                //RobotMap.mover2.set(ControlMode.PercentOutput, speed2*antiGravity);
            }
        } else {
            releaseArm();
        }
    }

    public static void report() {
        SmartDashboard.putNumber("mover1 ticks", RobotMap.mover1.getSelectedSensorPosition());
        SmartDashboard.putNumber("mover2 ticks", RobotMap.mover2.getSelectedSensorPosition());
    }

    public static void pidFollow() { 
        if(-RobotMap.secondController.getLeftY() > 0.5) {
            RobotMap.mover1.set(ControlMode.Position, testPosition); //test this before creating double controllers
            //RobotMap.mover2.set(ControlMode.Position, testPosition);
        }
    }



    public static void releaseArm() {
        if(releaseTimer.get() < 1) {
            RobotMap.release.set(0.75); //should turn the pg for 1s at 75 %
        } else {
            sensing = true;
            RobotMap.release.set(0); //turn the motor off
        }
    }

    public static void checkArm() { //TODO try to get a sensor here
        
    }

    static class FollowPID extends PIDController{ //would require 2 custom PID controllers on 1 motor
        public FollowPID(double kP, double kI, double kD, double maxSpeed, double setpoint) {
            super(kP, kI, kD);
        }
    }

}
