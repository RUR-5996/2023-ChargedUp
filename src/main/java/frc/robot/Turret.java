package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.math.MathUtil;

public class Turret {

    static final double LEFT_LIMIT = -13;
    static final double RIGHT_LIMIT = 6;
    static final double TURRET_TICKS_TO_DEGS = 10; //TODO napocitat spravne
    static final double LOWER_LIMIT = 0;
    static final double UPPER_LIMIT = 88.5+185; //ticks
    static final double HOOD_TICKS_TO_DEGS = 10;

    static double turretDegrees = 0;
    static double hoodDegrees = 0;
    static double holdAngle = 0;

    static boolean leftTurretLimit = false;
    static boolean rightTurretLimit = false;
    static boolean lowerHoodLimit = false;
    static boolean upperHoodLimit = false;

    static TurretPID turretPID;



    public static void turretInit() {
        RobotMap.lowerShooter.configFactoryDefault();
        RobotMap.lowerShooter.setInverted(false); //use constant
        RobotMap.lowerShooter.configOpenloopRamp(0.2);
        RobotMap.lowerShooter.configClosedloopRamp(0.2);
        RobotMap.lowerShooter.config_kP(0, 0.5);
        RobotMap.lowerShooter.config_kI(0, 0);
        RobotMap.lowerShooter.config_kD(0, 0);
        RobotMap.lowerShooter.setInverted(true);

        RobotMap.upperShooter.configFactoryDefault();
        RobotMap.upperShooter.setInverted(false); //use constant
        RobotMap.upperShooter.configOpenloopRamp(0.2);
        RobotMap.upperShooter.configClosedloopRamp(0.2);
        RobotMap.upperShooter.config_kP(0, 0.5);
        RobotMap.upperShooter.config_kI(0, 0);
        RobotMap.upperShooter.config_kD(0, 0);
        RobotMap.upperShooter.setInverted(true);

        RobotMap.feederMotor.configFactoryDefault();
        RobotMap.feederMotor.configOpenloopRamp(0.3);
        RobotMap.feederMotor.setInverted(true);

        RobotMap.upperShooter.follow(RobotMap.lowerShooter);

        RobotMap.turretMotor.restoreFactoryDefaults(); //will probably need nested PID controller
        RobotMap.turretMotor.setOpenLoopRampRate(0.5);
        RobotMap.hoodMotor.restoreFactoryDefaults();

        turretPID = new TurretPID(0.5, 0, 0, 0, 0.5);

    }

    public static void manualRotate() {
        if(RobotMap.secondController.getLeftX() > 0.5) {
            turretMotor(-0.2);
        } else if(RobotMap.secondController.getLeftX() < -0.5) {
            turretMotor(0.2);
        } else {
            turretMotor(0);
        }
    }

    public static void periodic() {
        manualRotate();
        manualHood();

        if(RobotMap.secondController.getLeftBumper()||RobotMap.controller.getLeftTriggerAxis() > 0.5) {
            shoot();
        } else {
            stop();
        }
    }

    public static void manualHood() {
        if(RobotMap.secondController.getLeftY() > 0.5) {
            hoodMotor(-0.5);
        } else if(RobotMap.secondController.getLeftY() < -0.5) {
            hoodMotor(0.5);
        } else {
            hoodMotor(0);
        }
    }

    public static void staticTurret() {
        turretPID.setOffset(turretDegrees);
        turretPID.setTarget(holdAngle - SwerveDef.gyro.getAngle());
        turretMotor(turretPID.pidGet());
    }

    public static void cameraTurret() {
        
    }

    public static void shoot() {
        RobotMap.lowerShooter.set(VictorSPXControlMode.PercentOutput, 0.65);
        RobotMap.upperShooter.set(VictorSPXControlMode.PercentOutput, 0.65);
        RobotMap.feederMotor.set(VictorSPXControlMode.PercentOutput, 0.4);
    }

    public static void stop() {
        RobotMap.lowerShooter.set(VictorSPXControlMode.PercentOutput, 0);
        RobotMap.upperShooter.set(VictorSPXControlMode.PercentOutput, 0);
        RobotMap.feederMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public static void robotPeriodic() {
        SmartDashboard.putNumber("hood ticks", RobotMap.hoodEncoder.getPosition());
        SmartDashboard.putNumber("turret ticks", RobotMap.turretEncoder.getPosition());
    }

    static void turretMotor(double speed) { //kladn8 rychlost doprava
        if((leftTurretLimit && speed > 0) || (rightTurretLimit && speed < 0) || (!leftTurretLimit && !rightTurretLimit)) {
            RobotMap.turretMotor.set(speed);
        } else {
            RobotMap.turretMotor.set(0);
        }
    }

    static void hoodMotor(double speed) {
        if((upperHoodLimit && speed > 0) || (lowerHoodLimit && speed < 0) || (!upperHoodLimit && !lowerHoodLimit)) {
            RobotMap.hoodMotor.set(speed);
        } else {
            RobotMap.hoodMotor.set(0);
        }
    }

    static class TurretPID extends PIDController{
        static double kP, kI, kD, holdAngle, maxSpeed, offset;
        static boolean holdingAnle = false;
        public TurretPID(double p, double i, double d, double angle, double mSpeed) {
            super(p, i, d);
            super.setTolerance(0.5);

            kP = p;
            kI = i;
            kD = d;
            holdAngle = angle;
            maxSpeed = mSpeed;
        }

        public void setTarget(double angle) { //turreta udrzuje bud konstatni nebo promenlivy uhel
            holdAngle = angle;
        }

        public void setOffset(double cOffset) {
            offset = cOffset;
        }

        public double pidGet() {
            double speed = MathUtil.clamp(super.calculate(offset, holdAngle), -maxSpeed, maxSpeed);
            return speed;
        }

    }

    static class DistanceCalculator {
        static double tY, tA, dist, angle, speed;

        public DistanceCalculator(double y, double a) {
            tY = y;
            tA = a;
        }

        static void calcDistance() {

        }

        static void calcAngle() {

        }

        static void calcSpeed() {
            
        }
    }
    
}
