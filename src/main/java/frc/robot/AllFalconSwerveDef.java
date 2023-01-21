package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;

/**
 * @deprecated really messy, but should be working just fine - just adjust PID values
 */
public class AllFalconSwerveDef {

    public static class SteerMotor extends WPI_TalonFX {
        public TalonFXInvertType invertType;

        public SteerMotor(int id, TalonFXInvertType invertType) {
            super(id);
            this.invertType = invertType;
        }
    }

    public static class DriveMotor extends WPI_TalonFX {
        public TalonFXInvertType invertType;

        public DriveMotor(int id, TalonFXInvertType invertType) {
            super(id);
            this.invertType = invertType;
        }
    }

    // Use this structure when using off-motor encoder. Implement in SwerveModule
    // class!
    public static class SteerSensor extends AnalogInput {
        public double offset;
        public AnalogInput sensor;

        public SteerSensor(int id, double offset) {
            super(id);
            this.offset = offset;
        }
    }

    // should create a lib for this
    public static class SwerveModule {
        public SteerMotor steerMotor;
        public DriveMotor driveMotor;
        public SteerSensor steerSensor;
        public pidValues drivePID, steerPID;

        public SwerveModule(SteerMotor sMotor, pidValues sPID, DriveMotor dMotor, pidValues dPID, SteerSensor sensor) {
            steerMotor = sMotor;
            driveMotor = dMotor;
            drivePID = dPID;
            steerPID = sPID;
            steerSensor = sensor;
        }

        public void moduleInit() {
            driveMotor.configFactoryDefault();
            driveMotor.setInverted(driveMotor.invertType);
            driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
            driveMotor.config_kP(0, drivePID.kP, TIMEOUT_MS);
            driveMotor.config_kI(0, drivePID.kI, TIMEOUT_MS);
            driveMotor.config_kD(0, drivePID.kD, TIMEOUT_MS);
            driveMotor.config_kF(0, drivePID.kF, TIMEOUT_MS);
            driveMotor.config_IntegralZone(0, 300); // do we need this?
            driveMotor.configOpenloopRamp(0.2);

            steerMotor.configFactoryDefault();
            steerMotor.configFeedbackNotContinuous(false, TIMEOUT_MS);
            steerMotor.configSelectedFeedbackCoefficient(1 / STEER_FEEDBACK_COEFFICIENT, 0, TIMEOUT_MS);
            steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
            steerMotor.config_kP(0, steerPID.kP, TIMEOUT_MS);
            steerMotor.config_kI(0, steerPID.kI, TIMEOUT_MS);
            steerMotor.config_kD(0, steerPID.kD, TIMEOUT_MS);
            steerMotor.config_kF(0, steerPID.kF, TIMEOUT_MS);
            steerMotor.configAllowableClosedloopError(1, 0, TIMEOUT_MS);
            steerMotor.configOpenloopRamp(0.2);
            zeroSwerve();
        }

        public void disabledInit() {
            driveMotor.setNeutralMode(NeutralMode.Coast);
            steerMotor.setNeutralMode(NeutralMode.Coast);
        }

        public void enabledInit() {
            driveMotor.setNeutralMode(NeutralMode.Brake);
            steerMotor.setNeutralMode(NeutralMode.Brake);
        }

        public void zeroSwerve() {
            // double supposedZero = (getBetterAnalogDegs() - steerSensor.offset) *
            // STEER_FEEDBACK_COEFFICIENT;
            double supposedZero = 0;
            /*
             * steerMotor.setSelectedSensorPosition(
             * steerSensor.getVoltage() * STEER_SENSOR_COEFF_TO_DEG - steerSensor.offset, 0,
             * TIMEOUT_MS);
             */
            steerMotor.setSelectedSensorPosition(clampContinuousDegs(supposedZero), 0, TIMEOUT_MS);
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    driveMotor.getSelectedSensorVelocity(0) * DRIVE_DIST_PER_WHEEL_REV
                            / (0.1 * DRIVE_TICKS_PER_MOTOR_REV),
                    new Rotation2d(Math.toRadians(steerMotor.getSelectedSensorPosition())));
        }

        public SwerveModuleState optimizeState(SwerveModuleState stateToOptimize, Rotation2d moduleAngle) {
            Rotation2d diff = stateToOptimize.angle.minus(moduleAngle);

            if (Math.abs(diff.getDegrees()) > 90) {
                return new SwerveModuleState(-stateToOptimize.speedMetersPerSecond,
                        stateToOptimize.angle.rotateBy(Rotation2d.fromDegrees(180)));
            } else {
                return new SwerveModuleState(stateToOptimize.speedMetersPerSecond, stateToOptimize.angle);
            }
        }

        public void setAngle(double angleToSet) {
            double encoderPosition = steerMotor.getSelectedSensorPosition();
            double toFullCircle = Math.IEEEremainder(encoderPosition, 360);
            double newAngle = angleToSet + encoderPosition - toFullCircle;

            if (newAngle - encoderPosition > 180.1) {
                newAngle -= 360;
            } else if (newAngle - encoderPosition < -180.1) {
                newAngle += 360;
            }
            steerMotor.set(ControlMode.Position, newAngle); // runs PID
        }

        public void setState(SwerveModuleState stateToSet) {
            SwerveModuleState optimizedState = optimizeState(stateToSet,
                    new Rotation2d(Math.toRadians(steerMotor.getSelectedSensorPosition())));
            // double speed =
            // optimizedState.speedMetersPerSecond*(0.1*DRIVE_TICKS_PER_MOTOR_REV/(DRIVE_DIST_PER_WHEEL_REV));
            // probably not needed

            setAngle(optimizedState.angle.getDegrees());

            driveMotor.set(ControlMode.PercentOutput, optimizedState.speedMetersPerSecond / MAX_SPEED_MPS);
        }

        public double getBetterAnalogDegs() {
            if (steerSensor.getVoltage() == 2.5) {
                return 0.0;
            } else if (steerSensor.getVoltage() < 2.5) {
                return (2.5 - steerSensor.getVoltage()) * STEER_SENSOR_COEFF_TO_DEG;
            } else if (steerSensor.getVoltage() > 2.5) {
                return (2.5 - steerSensor.getVoltage()) * STEER_SENSOR_COEFF_TO_DEG;
            } else {
                return 0;
            }
        }

        public double clampContinuousDegs(double toClamp) {
            if (toClamp < -180) {
                return toClamp + 360.0;
            } else if (toClamp > 180) {
                return toClamp - 360.0;
            } else {
                return toClamp;
            }
        }

    }

    // compact way to stiore PID controller constants
    public static class pidValues {
        public double kP;
        public double kI;
        public double kD;
        public double kF;

        public pidValues(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = 0;
        }

        public pidValues(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

    public static class initPID extends PIDController {
        private double target, offset, maxSpeed;

        public initPID(double kP, double kI, double kD, double mSpeed, double setpoint) {
            super(kP, kI, kD);
            super.setTolerance(0.72);
            super.enableContinuousInput(-180, 180);
            maxSpeed = mSpeed;
            target = setpoint;
        }

        public void setTarget(double setpoint) {
            target = setpoint;
        }

        public void setOffset(double value) {
            offset = value;
        }

        public double pidGet() {
            double speed = MathUtil.clamp(super.calculate(offset, target), -maxSpeed, maxSpeed);
            return speed;
        }

    }

    public static AHRS gyro = new AHRS(SPI.Port.kMXP);

    public static final TalonFXInvertType FL_STEER_INVERT_TYPE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType FR_STEER_INVERT_TYPE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType RL_STEER_INVERT_TYPE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType RR_STEER_INVERT_TYPE = TalonFXInvertType.Clockwise;

    public static final TalonFXInvertType FL_DRIVE_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType FR_DRIVE_INVERT_TYPE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType RL_DRIVE_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType RR_DRIVE_INVERT_TYPE = TalonFXInvertType.Clockwise;

    public static final pidValues FL_STEER_PID_VALUES = new pidValues(9, 0, 0);
    public static final pidValues FR_STEER_PID_VALUES = new pidValues(9, 0, 0);
    public static final pidValues RL_STEER_PID_VALUES = new pidValues(9, 0, 0);
    public static final pidValues RR_STEER_PID_VALUES = new pidValues(9, 0, 0);

    public static final pidValues FL_DRIVE_PID_VALUES = new pidValues(0, 0, 0);
    public static final pidValues FR_DRIVE_PID_VALUES = new pidValues(0, 0, 0);
    public static final pidValues RL_DRIVE_PID_VALUES = new pidValues(0, 0, 0);
    public static final pidValues RR_DRIVE_PID_VALUES = new pidValues(0, 0, 0);

    public static final double FL_STEER_OFFSET = 118.7 - 5.76;
    public static final double FL_STEER_TURNAROUND = 0; // *1
    public static final double FL_STEER_COEFF = 1;
    public static final double FR_STEER_OFFSET = 111.1 - 1.14 + 15.8;
    public static final double FR_STEER_TURNAROUND = -180; // *1
    public static final double FR_STEER_COEFF = 1;
    public static final double RL_STEER_OFFSET = 20.05 + 1.65;
    public static final double RL_STEER_TURNAROUND = 0; // *-1
    public static final double RL_STEER_COEFF = -1;
    public static final double RR_STEER_OFFSET = 124.2 - 3.17;
    public static final double RR_STEER_TURNAROUND = -180;
    public static final double RR_STEER_COEFF = 1;

    public static final double FALCON_TICKS_PER_MOTOR_REV = 2048.0;
    public static final double STEER_SENSOR_COEFF_TO_DEG = 360.0 / 5.0;
    public static final double MAX_SPEED_TICKS_100_MS = 21900;
    public static final double DRIVE_MOTOR_GEARING = 6.92;
    public static final double DRIVE_COEFFICIENT = 0.3;
    public static final double TURN_COEFFICIENT = 0.5;
    public static final double WHEEL_RADIUS = 0.05138; // m
    public static final double MAX_WHEEL_SPEED = 0.25;
    public static final double WHEEL_BASE_WIDTH = 0.45;
    public static final double TRACK_WIDTH = 0.44;
    public static final double DRIVE_DIST_PER_WHEEL_REV = 2 * Math.PI * WHEEL_RADIUS; // m
    public static final double DRIVE_DIST_PER_ROBOT_REV = 2 * Math.PI
            * Math.sqrt(Math.pow(WHEEL_BASE_WIDTH, 2) + Math.pow(TRACK_WIDTH, 2));
    public static final double MAX_SPEED_MPS = MAX_SPEED_TICKS_100_MS / 0.1 / FALCON_TICKS_PER_MOTOR_REV
            * DRIVE_DIST_PER_WHEEL_REV;
    public static final double MAX_SPEED_RADPS = MAX_SPEED_MPS / DRIVE_DIST_PER_ROBOT_REV * (2 * Math.PI);

    public static final double STEER_FEEDBACK_COEFFICIENT = FALCON_TICKS_PER_MOTOR_REV * 18.0 / 360.0; // think of an
    public static final double DRIVE_TICKS_PER_MOTOR_REV = FALCON_TICKS_PER_MOTOR_REV * 10; // TICKS PER MOTOR REV*DRIVE
                                                                                            // GEAR RATIO
    public static final int TIMEOUT_MS = 20;

    public static SteerMotor flSteer = new SteerMotor(5, FL_STEER_INVERT_TYPE);
    public static DriveMotor flDrive = new DriveMotor(3, FL_DRIVE_INVERT_TYPE);
    public static SteerSensor flSensor = new SteerSensor(1, FL_STEER_OFFSET);
    public static SwerveModule flModule = new SwerveModule(flSteer, FL_STEER_PID_VALUES, flDrive, FL_DRIVE_PID_VALUES,
            flSensor);

    public static SteerMotor frSteer = new SteerMotor(8, FR_STEER_INVERT_TYPE);
    public static DriveMotor frDrive = new DriveMotor(0, FR_DRIVE_INVERT_TYPE);
    public static SteerSensor frSensor = new SteerSensor(3, FR_STEER_OFFSET);
    public static SwerveModule frModule = new SwerveModule(frSteer, FR_STEER_PID_VALUES, frDrive, FR_DRIVE_PID_VALUES,
            frSensor);

    public static SteerMotor rlSteer = new SteerMotor(7, RL_STEER_INVERT_TYPE);
    public static DriveMotor rlDrive = new DriveMotor(2, RL_DRIVE_INVERT_TYPE);
    public static SteerSensor rlSensor = new SteerSensor(0, RL_STEER_OFFSET);
    public static SwerveModule rlModule = new SwerveModule(rlSteer, RL_STEER_PID_VALUES, rlDrive, RL_DRIVE_PID_VALUES,
            rlSensor);

    public static SteerMotor rrSteer = new SteerMotor(4, RR_STEER_INVERT_TYPE);
    public static DriveMotor rrDrive = new DriveMotor(1, RR_DRIVE_INVERT_TYPE);
    public static SteerSensor rrSensor = new SteerSensor(2, RR_STEER_OFFSET);
    public static SwerveModule rrModule = new SwerveModule(rrSteer, RR_STEER_PID_VALUES, rrDrive, RR_DRIVE_PID_VALUES,
            rrSensor);
}