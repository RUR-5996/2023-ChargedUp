package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

    //single instance
    private static SwerveDrive SWERVE = new SwerveDrive();

    //control variables
    static XboxController controller = RobotMap.controller;
    static final double MF_DRIVE_COEFF = 0.5; // change to lower if needed
    static double addDriveCoeff = 1;

    private static double xSpeed = 0;
    private static double ySpeed = 0;
    private static double rotation = 0;

    public static boolean fieldOriented = false;
    public static double holdAngle = 0;
    public static boolean fieldRelative = false; 

    //module position definition
    static final Translation2d FL_LOC = new Translation2d(SwerveDef.WHEEL_BASE_WIDTH / 2, SwerveDef.TRACK_WIDTH / 2);
    static final Translation2d FR_LOC = new Translation2d(SwerveDef.WHEEL_BASE_WIDTH / 2, -SwerveDef.TRACK_WIDTH / 2);
    static final Translation2d RL_LOC = new Translation2d(-SwerveDef.WHEEL_BASE_WIDTH / 2, SwerveDef.TRACK_WIDTH / 2);
    static final Translation2d RR_LOC = new Translation2d(-SwerveDef.WHEEL_BASE_WIDTH / 2, -SwerveDef.TRACK_WIDTH / 2);
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(FL_LOC, FR_LOC, RL_LOC, RR_LOC);
    public static SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    public static PIDController angleHoldController = new PIDController(0.1, 0, 0); // edit the vals
    public static SwerveDriveOdometry odometry;

    //PID definitions
    //TODO use built-in controllers for module init
    //public static SwerveDef.SteerPID flInitController;
    //public static SwerveDef.SteerPID frInitController;
    //public static SwerveDef.SteerPID rlInitController;
    //public static SwerveDef.SteerPID rrInitController;

    /**
     * Function for getting the single instance of this class
     * @return SwerveDrive instance
     */
    public static SwerveDrive getInstance() {
        return SWERVE;
    }

    /**
     * Function for setting up the SwerveDrive object
     */
    public void init() {
        updateModulePosition();
        initFieldOriented();
        odometry = new SwerveDriveOdometry(swerveKinematics, SwerveDef.gyro.getRotation2d(), modulePositions);
        angleHoldController.disableContinuousInput();
        angleHoldController.setTolerance(Math.toRadians(2)); // the usual drift
        testInit();
    }

    /**
     * Function for properly updating the current module position
     * TODO could be done in SwerveDef for cleaner code
     */
    public static void updateModulePosition() {
        modulePositions[0] = new SwerveModulePosition(SwerveDef.flModule.getState().speedMetersPerSecond, SwerveDef.flModule.getState().angle);
        modulePositions[1] = new SwerveModulePosition(SwerveDef.frModule.getState().speedMetersPerSecond, SwerveDef.frModule.getState().angle);
        modulePositions[2] = new SwerveModulePosition(SwerveDef.rlModule.getState().speedMetersPerSecond, SwerveDef.rlModule.getState().angle);
        modulePositions[3] = new SwerveModulePosition(SwerveDef.rrModule.getState().speedMetersPerSecond, SwerveDef.rrModule.getState().angle);
    }

    /**
     * Function defining basic fieldOriented behavior
     * sets holdAngle
     */
    public static void initFieldOriented() {
        fieldRelative = true;
        holdAngle = SwerveDef.gyro.getRotation2d().getRadians();
    }

    /**
     * Function for setting default driving orentation
     * Defaulted to FieldRelative driving
     */
    public static void initRobotOriented() {
        fieldRelative = true;
    }

    /**
     * Function with the decision tree for driving the chassis
     */
    public static void periodic() {

        //drive(0, 0, 0);
        drive();
        //orientedDrive();
        report();
    }

    /**
     * Function for setting up the speeds of the modules based on controller input and state optimization
     */
    public static void drive() {
        xSpeed = deadzone(-controller.getLeftX()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT * addDriveCoeff;
        ySpeed = deadzone(-controller.getLeftY()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT * addDriveCoeff;
        rotation = deadzone(-controller.getRightX()) * SwerveDef.MAX_SPEED_RADPS * SwerveDef.TURN_COEFFICIENT * addDriveCoeff; 

        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(ySpeed, xSpeed, rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDef.MAX_SPEED_MPS);

        SwerveDef.flModule.setState(states[0]);
        SwerveDef.frModule.setState(states[1]);
        SwerveDef.rlModule.setState(states[2]);
        SwerveDef.rrModule.setState(states[3]);
    }

    /**
     * Function for setting module speeds based on given input (semi)autonomous
     * @param xSpeed
     * @param ySpeed
     * @param rotation
     */
    public static void drive(double xSpeed, double ySpeed, double rotation) {
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(ySpeed, xSpeed, rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDef.MAX_SPEED_MPS);

        SwerveDef.flModule.setState(states[0]);
        SwerveDef.frModule.setState(states[1]);
        SwerveDef.rlModule.setState(states[2]);
        SwerveDef.rrModule.setState(states[3]);
    }

    /**
     * Function for setting module speeds based on controller input during field oriented driving
     */
    public static void orientedDrive() {
        xSpeed = deadzone(-controller.getLeftX()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT * addDriveCoeff;
        ySpeed = deadzone(-controller.getLeftY()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT * addDriveCoeff;
        rotation = deadzone(-controller.getRightX()) * SwerveDef.MAX_SPEED_RADPS * SwerveDef.TURN_COEFFICIENT * addDriveCoeff;

        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, SwerveDef.gyro.getRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDef.MAX_SPEED_MPS);

        SwerveDef.flModule.setState(states[0]);
        SwerveDef.frModule.setState(states[1]);
        SwerveDef.rlModule.setState(states[2]);
        SwerveDef.rrModule.setState(states[3]);
    }

    /**
     * Function for creating custom deadzone around joystick axis
     * @param input raw axis value
     * @return filtered joystick input
     */
    public static double deadzone(double input) {
        if (Math.abs(input) < 0.2) {
            return 0;
        } else {
            return input;
        }
    }

    /**
     * @deprecated sets module rotation to 0
     * should be used with relative encoders
     */
    public static void resetZero() {
        if (controller.getAButtonPressed()) {
            SwerveDef.flModule.zeroSwerve();
            SwerveDef.frModule.zeroSwerve();
            SwerveDef.rlModule.zeroSwerve();
            SwerveDef.rrModule.zeroSwerve();
        }
    }

    /**
     * Function for starting the calibration routine of the swerve modules
     */
    public static void testInit() {
        /*flInitController = new SwerveDef.SteerPID(0.006, 0, 0, 1, 0);
        frInitController = new SwerveDef.SteerPID(0.006, 0, 0, 1, 0);
        rlInitController = new SwerveDef.SteerPID(0.006, 0, 0, 1, 0);
        rrInitController = new SwerveDef.SteerPID(0.006, 0, 0, 1, 0);*/
    }

    /**
     * Function for homing all the modules
     * TODO check, if this works with absolute encoders
     */
    public static void zeroDrive() {
        /*flInitController.setOffset(SwerveDef.flModule.clampContinuousDegs(SwerveDef.flModule.getBetterAnalogDegs()));//TODO use built-in functions instead

        frInitController.setOffset(SwerveDef.frModule.clampContinuousDegs(SwerveDef.frModule.getBetterAnalogDegs()));

        rlInitController.setOffset(SwerveDef.rlModule.clampContinuousDegs(SwerveDef.rlModule.getBetterAnalogDegs()));

        rrInitController.setOffset(SwerveDef.rrModule.clampContinuousDegs(SwerveDef.rrModule.getBetterAnalogDegs()));

        SwerveDef.flSteer.set(ControlMode.PercentOutput, flInitController.pidGet());
        SwerveDef.frSteer.set(ControlMode.PercentOutput, frInitController.pidGet());
        SwerveDef.rlSteer.set(ControlMode.PercentOutput, rlInitController.pidGet());
        SwerveDef.rrSteer.set(ControlMode.PercentOutput, rrInitController.pidGet());*/
    }

    /**
     * Function for zeroing gyroscope heading
     * Should not be used much during competition, takes long-ish time
     */
    static void gyroReset() {
        SwerveDef.gyro.reset();
    }

    /**
     * Function for updating chassis odometry
     */
    static void updateOdometry() {
        odometry.update(SwerveDef.gyro.getRotation2d(), modulePositions);
    }

    /**
     * Function for reseting chassis odometry
     */
    static void resetOdometry() {
        odometry.resetPosition(SwerveDef.gyro.getRotation2d(), modulePositions, new Pose2d() );
    }

    /**
     * Function for setting custom chassis odometry (start of autonomous)
     * @param pose
     * @param rot
     */
    static void setOdometry(Pose2d pose, Rotation2d rot) {
        odometry.resetPosition(rot, modulePositions, pose);
    }

    public static void report() {
        SmartDashboard.putNumber("FL encoder", SwerveDef.flModule.clampContinuousDegs());
        SmartDashboard.putNumber("FR encoder", SwerveDef.flModule.clampContinuousDegs());
        SmartDashboard.putNumber("RL encoder", SwerveDef.flModule.clampContinuousDegs());
        SmartDashboard.putNumber("RR encoder", SwerveDef.flModule.clampContinuousDegs());

        SmartDashboard.putNumber("gyro angle", SwerveDef.gyro.getAngle());
    }
}
