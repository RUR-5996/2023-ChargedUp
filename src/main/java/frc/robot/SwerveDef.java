package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDef {

    /**
     * SteerMotor object defined on top of VictorSPX structure (brushed motor)
     */
    public static class SteerMotor extends WPI_VictorSPX {
        public boolean inverted;

        /**
         * SteerMotor constructor
         * @param id VictorSPX id on CAN bus
         * @param inverted sets the direction of rotation based on default input
         */
        public SteerMotor(int id, boolean inverted) {
            super(id);
            this.inverted = inverted;
        }
    }

    /**
     * DriveMotor object defined on top of TalonFX structure (Falcon motor)
     */
    public static class DriveMotor extends WPI_TalonFX {
        public TalonFXInvertType invertType;

        /**
         * DriveMotor constructor
         * @param id TalonFX id on CAN bus
         * @param invertType sets the direction of rotation based on default input
         */
        public DriveMotor(int id, TalonFXInvertType invertType) {
            super(id);
            this.invertType = invertType;
        }
    }

    /**
     * SteerSensor object defined on top of AnalogInput structure
     * defines absolute analog encoder for ALL steering purposes
     */
    public static class SteerSensor extends AnalogInput {
        public double offset;
        public AnalogInput sensor;

        /**
         * SteerSensor constructor
         * @param id AnalogInput port on RoboRIO
         * @param offset rotational offset from ideal 0 heading (probably degrees)
         */
        public SteerSensor(int id, double offset) {
            super(id);
            this.offset = offset; //TODO confirm if this is degs
        }
    }

    /**
     * SwerveModule object that does all the swerving and steering
     * blackbox for now, will elaborate (hopefully)
    */
    public static class SwerveModule {
        public SteerMotor steerMotor;
        public DriveMotor driveMotor;
        public SteerSensor steerSensor;
        public PidValues drivePID, steerPID;
        public SteerPID steerPIDController;

        /**
         * SwerveModule constructor
         * @param sMotor SteerMotor object for given module
         * @param sPID  PidValues object for the SteerMotor
         * @param dMotor DriveMotor object for given module
         * @param dPID PidValues object for the DriveMotor
         * @param sensor SteerSensor object for given module
         */
        public SwerveModule(SteerMotor sMotor, PidValues sPID, DriveMotor dMotor, PidValues dPID, SteerSensor sensor) {
            steerMotor = sMotor;
            driveMotor = dMotor;
            drivePID = dPID;
            steerPID = sPID;
            steerSensor = sensor;

            steerPIDController = new SteerPID(steerPID.kP, steerPID.kI, steerPID.kD, 0.7);
        }

        /**
         * function for setting up the module itself (the motors to be specific)
         * should run at robotInit()
         * zeros the swerve, has to be overwritten
         * TODO find a way to do better swerve homing
         */
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
            steerMotor.setInverted(steerMotor.inverted);
            steerMotor.config_kP(0, steerPID.kP, TIMEOUT_MS);
            steerMotor.config_kI(0, steerPID.kI, TIMEOUT_MS);
            steerMotor.config_kD(0, steerPID.kD, TIMEOUT_MS);
            steerMotor.config_kF(0, steerPID.kF, TIMEOUT_MS);
            steerMotor.config_IntegralZone(0, 300);
            steerMotor.configAllowableClosedloopError(1, 0, TIMEOUT_MS);
            steerMotor.configOpenloopRamp(0.2);
            
            //zeroSwerve(); //might not be necessary - steering runs on abs encoders TODO look into this
        }

        /**
         * Function defining motor behavior on disabled 
         * should consider .Brake for comp to minimize sliding around
         */
        public void disabledInit() {
            driveMotor.setNeutralMode(NeutralMode.Coast);
            steerMotor.setNeutralMode(NeutralMode.Coast);
        }

        /**
         * Function defining motor behavior on enabled
         * should always be braking to avoid drifting
         */
        public void enabledInit() {
            driveMotor.setNeutralMode(NeutralMode.Brake);
            steerMotor.setNeutralMode(NeutralMode.Brake);
        }

        /**
         * @deprecated, should be brought back, if using relative encoders.
         */
        public void zeroSwerve() {
            double supposedZero = 0;
            steerMotor.setSelectedSensorPosition(clampContinuousDegs(supposedZero), 0, TIMEOUT_MS);
        }

        /**
         * Function for determinig current module state (position, rotation)
         * @return SwerveModuleState that gets processed down the line
         */
        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    driveMotor.getSelectedSensorVelocity(0) * DRIVE_DIST_PER_WHEEL_REV / (0.1 * DRIVE_TICKS_PER_MOTOR_REV),
                    new Rotation2d(Math.toRadians(steerMotor.getSelectedSensorPosition())));
        }

        /**
         * Function optimizing desired SwerveState (adjusts angles and respective travel direction)
         * @param stateToOptimize desired state calculated by Swerve controller
         * @param moduleAngle current angle of the module
         * @return SwerveModuleState with optimized rotation pathing
         */
        public SwerveModuleState optimizeState(SwerveModuleState stateToOptimize, Rotation2d moduleAngle) {
            Rotation2d diff = stateToOptimize.angle.minus(moduleAngle);

            if (Math.abs(diff.getDegrees()) > 90) {
                return new SwerveModuleState(-stateToOptimize.speedMetersPerSecond,
                        stateToOptimize.angle.rotateBy(Rotation2d.fromDegrees(180)));
            } else {
                return new SwerveModuleState(stateToOptimize.speedMetersPerSecond, stateToOptimize.angle);
            }
        }

        /**
         * Function that calculates direction of travel and tells the steer motor to set desired optimized angle of the module
         * sets PID target and the controller covers the speeds
         * @param angleToSet double new angle in degrees
         */
        public void setAngle(double angleToSet) {
            double encoderPosition = clampContinuousDegs(getBetterAnalogDegs());
            double toFullCircle = Math.IEEEremainder(encoderPosition, 360);
            double newAngle = angleToSet + encoderPosition - toFullCircle; //TODO check the math, should be fine though

            if (newAngle - encoderPosition > 180.1) {
                newAngle -= 360;
            } else if (newAngle - encoderPosition < -180.1) {
                newAngle += 360;
            }
            
            steerPIDController.setTarget(newAngle);
            steerPIDController.setOffset(encoderPosition);
            steerMotor.set(ControlMode.PercentOutput, steerPIDController.pidGet());
        }

        /**
         * Function that sets the speed on drive motor
         * @param stateToSet SwerveModuleState optimized module state
         */
        public void setState(SwerveModuleState stateToSet) {
            SwerveModuleState optimizedState = optimizeState(stateToSet, new Rotation2d(Math.toRadians(steerMotor.getSelectedSensorPosition())));

            setAngle(optimizedState.angle.getDegrees());

            driveMotor.set(ControlMode.PercentOutput, optimizedState.speedMetersPerSecond / MAX_SPEED_MPS);
        }

        /**
         * Function converting SteerSensor voltage to degrees for easier control
         * @return current module angle in degrees
         */
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

        /**
         * Function clamping module angle reading between -180 and 180 degrees
         * @param toClamp current encoder reading in degrees
         * @return current module angle between -180 and 180 degrees
         */
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

    /**
     * class for easy PID values storing and access
     */
    public static class PidValues {
        public double kP;
        public double kI;
        public double kD;
        public double kF;

        /**
         * constructor for PidValues object using PID control
         * @param kP
         * @param kI
         * @param kD
         */
        public PidValues(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = 0;
        }

        /**
         * constructor for PidValues object using PIDF control
         * @param kP
         * @param kI
         * @param kD
         * @param kF
         */
        public PidValues(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

    /**
     * InitPID object built on PIDController structure
     * Modified to make setting offsets, targets and values easier
     */
    public static class SteerPID extends PIDController {
        private double target, offset, maxSpeed;

        /**
         * InitPID constructor
         * @param kP
         * @param kI
         * @param kD
         * @param mSpeed maximal allowed speed of motion in percent
         * @param setpoint desired destination point
         */
        public SteerPID(double kP, double kI, double kD, double mSpeed, double setpoint) {
            super(kP, kI, kD);
            super.setTolerance(0.72);
            super.enableContinuousInput(-180, 180); //used of rotary things (robot rotation, swerve module rotation)
            maxSpeed = mSpeed;
            target = setpoint;
        }

        public SteerPID(double kP, double kI, double kD, double mSpeed) {
            super(kP, kI, kD);
            super.setTolerance(0.72);
            super.enableContinuousInput(-180, 180);
            maxSpeed = mSpeed;
            target = 0;
        }

        /**
         * Function for setting new target for the PID controller
         * @param setpoint desired destination point
         */
        public void setTarget(double setpoint) {
            target = setpoint;
        }

        /**
         * Function for setting current position of mechanism
         * @param value current position given by sensors
         */
        public void setOffset(double value) {
            offset = value;
        }

        /**
         * Function for calculating speed of travel to get to desired destination point
         * @return motor speed in percent
         */
        public double pidGet() {
            double speed = MathUtil.clamp(super.calculate(offset, target), -maxSpeed, maxSpeed);
            return speed;
        }

    }

    //NavX-MXP gyro definition
    public static AHRS gyro = new AHRS(SPI.Port.kMXP);

    //motor inverting
    public static final boolean FL_STEER_INVERTED = false;
    public static final boolean FR_STEER_INVERTED = false;
    public static final boolean RL_STEER_INVERTED = false;
    public static final boolean RR_STEER_INVERTED = false;

    public static final TalonFXInvertType FL_DRIVE_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType FR_DRIVE_INVERT_TYPE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType RL_DRIVE_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType RR_DRIVE_INVERT_TYPE = TalonFXInvertType.Clockwise;

    //setting up PID values (should use Sysid for precise measurements)
    //TODO consider advantages of PIDF controllers
    public static final PidValues FL_STEER_PID_VALUES = new PidValues(9, 0, 0);
    public static final PidValues FR_STEER_PID_VALUES = new PidValues(9, 0, 0);
    public static final PidValues RL_STEER_PID_VALUES = new PidValues(9, 0, 0);
    public static final PidValues RR_STEER_PID_VALUES = new PidValues(9, 0, 0);

    public static final PidValues FL_DRIVE_PID_VALUES = new PidValues(0, 0, 0);
    public static final PidValues FR_DRIVE_PID_VALUES = new PidValues(0, 0, 0);
    public static final PidValues RL_DRIVE_PID_VALUES = new PidValues(0, 0, 0);
    public static final PidValues RR_DRIVE_PID_VALUES = new PidValues(0, 0, 0);

    //general swerve control definition
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

    //general swerve mechanical definition
    //TODO check if actually using abs encoder for steering
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
    public static final double DRIVE_DIST_PER_ROBOT_REV = 2 * Math.PI * Math.sqrt(Math.pow(WHEEL_BASE_WIDTH, 2) + Math.pow(TRACK_WIDTH, 2));
    public static final double MAX_SPEED_MPS = MAX_SPEED_TICKS_100_MS / 0.1 / FALCON_TICKS_PER_MOTOR_REV * DRIVE_DIST_PER_WHEEL_REV;
    public static final double MAX_SPEED_RADPS = MAX_SPEED_MPS / DRIVE_DIST_PER_ROBOT_REV * (2 * Math.PI);

    public static final double STEER_FEEDBACK_COEFFICIENT = FALCON_TICKS_PER_MOTOR_REV * 18.0 / 360.0; // think of an
    public static final double DRIVE_TICKS_PER_MOTOR_REV = FALCON_TICKS_PER_MOTOR_REV * 10; // TICKS PER MOTOR REV*DRIVE GEAR RATIO

    //motor setup constants
    public static final int TIMEOUT_MS = 20;

    //FL module definition
    public static SteerMotor flSteer = new SteerMotor(5, FL_STEER_INVERTED);
    public static DriveMotor flDrive = new DriveMotor(3, FL_DRIVE_INVERT_TYPE);
    public static SteerSensor flSensor = new SteerSensor(1, FL_STEER_OFFSET);
    public static SwerveModule flModule = new SwerveModule(flSteer, FL_STEER_PID_VALUES, flDrive, FL_DRIVE_PID_VALUES, flSensor);

    //FR module definition
    public static SteerMotor frSteer = new SteerMotor(8, FR_STEER_INVERTED);
    public static DriveMotor frDrive = new DriveMotor(0, FR_DRIVE_INVERT_TYPE);
    public static SteerSensor frSensor = new SteerSensor(3, FR_STEER_OFFSET);
    public static SwerveModule frModule = new SwerveModule(frSteer, FR_STEER_PID_VALUES, frDrive, FR_DRIVE_PID_VALUES, frSensor);

    //RL module definition
    public static SteerMotor rlSteer = new SteerMotor(7, RL_STEER_INVERTED);
    public static DriveMotor rlDrive = new DriveMotor(2, RL_DRIVE_INVERT_TYPE);
    public static SteerSensor rlSensor = new SteerSensor(0, RL_STEER_OFFSET);
    public static SwerveModule rlModule = new SwerveModule(rlSteer, RL_STEER_PID_VALUES, rlDrive, RL_DRIVE_PID_VALUES, rlSensor);

    //RR module definition
    public static SteerMotor rrSteer = new SteerMotor(4, RR_STEER_INVERTED);
    public static DriveMotor rrDrive = new DriveMotor(1, RR_DRIVE_INVERT_TYPE);
    public static SteerSensor rrSensor = new SteerSensor(2, RR_STEER_OFFSET);
    public static SwerveModule rrModule = new SwerveModule(rrSteer, RR_STEER_PID_VALUES, rrDrive, RR_DRIVE_PID_VALUES, rrSensor);
}