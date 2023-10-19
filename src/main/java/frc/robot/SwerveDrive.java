package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
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

    private static double xSpeed = 0;
    private static double ySpeed = 0;
    private static double rotation = 0;

    public static double forward = 0;
    public static double sideways = 0;

    public static double limelightAimRotation = 0;

    public static boolean fieldOriented = false;
    public static double holdAngle = Math.PI/2;
    public static boolean fieldRelative = false; 

    public static boolean assistedDrive = false;

    public static boolean rampToggle = false;
    public static boolean slowmode = false;

    public static assistPID pid = new assistPID(0.1, 0, 0, 0);

    //module position definition
    static final Translation2d FL_LOC = new Translation2d(SwerveDef.WHEEL_BASE_WIDTH / 2, SwerveDef.TRACK_WIDTH / 2);
    static final Translation2d FR_LOC = new Translation2d(SwerveDef.WHEEL_BASE_WIDTH / 2, -SwerveDef.TRACK_WIDTH / 2);
    static final Translation2d RL_LOC = new Translation2d(-SwerveDef.WHEEL_BASE_WIDTH / 2, SwerveDef.TRACK_WIDTH / 2);
    static final Translation2d RR_LOC = new Translation2d(-SwerveDef.WHEEL_BASE_WIDTH / 2, -SwerveDef.TRACK_WIDTH / 2);
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(FL_LOC, FR_LOC, RL_LOC, RR_LOC);
    public static SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    public static PIDController angleHoldController = new PIDController(0.01, 0, 0); // edit the vals
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
        //SwerveDef.gyro.setAngleAdjustment(90);
        updateModulePosition();
        odometry = new SwerveDriveOdometry(swerveKinematics, SwerveDef.gyro.getRotation2d(), modulePositions);
        initFieldOriented();
        angleHoldController.disableContinuousInput();
        angleHoldController.setTolerance(Math.toRadians(2)); // the usual drift

        SwerveDef.flDrive.setNeutralMode(NeutralMode.Brake);
        SwerveDef.frDrive.setNeutralMode(NeutralMode.Brake);
        SwerveDef.rlDrive.setNeutralMode(NeutralMode.Brake);
        SwerveDef.rrDrive.setNeutralMode(NeutralMode.Brake);
        //testInit();
        
    }

    public static void ifEndMatch() {
        if(controller.getAButtonPressed()){
            SwerveDef.rlModule.setAngle(45);
            SwerveDef.rrModule.setAngle(135);
            SwerveDef.frModule.setAngle(45);
            SwerveDef.flModule.setAngle(135);
        }
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

    public static void speedMode() {
        if(RobotMap.secondController.getLeftStickButtonReleased() || RobotMap.secondController.getRightStickButtonReleased()) {
            slowmode = !slowmode;
        }
        SmartDashboard.putBoolean("slow Mode", slowmode);
    }

    /**
     * Function defining basic fieldOriented behavior
     * sets holdAngle
     */
    public static void initFieldOriented() {
        fieldRelative = true;
        holdAngle = SwerveDef.gyro.getRotation2d().getRadians() - Math.PI/2;
    }

    /**
     * Function for setting default driving orentation
     * Defaulted to FieldRelative driving
     */
    public static void initRobotOriented() {
        fieldRelative = false;
    }

    /**
     * Function with the decision tree for driving the chassis
     */
    public static void periodic() {

        //drive(0, 0, 0);
        speedMode(); //for demostration purposes only

        updateModulePosition();
        if(assistedDrive) {
            //assistedDrive(); TODO enable this later
        } else {
            orientedDrive();
        }
        gyroReset();
        ifEndMatch();

        //rampToggle = RobotMap.controller.getXButtonPressed();
        report();
    }

    public static void test() {
        SwerveDef.frModule.setAngle(+50);
    }

    /**
     * Function for setting up the speeds of the modules based on controller input and state optimization
     */
    public static void drive() {
        
        xSpeed = deadzone(controller.getLeftX()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        ySpeed = deadzone(controller.getLeftY()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        rotation = deadzone(controller.getRightX()) * SwerveDef.MAX_SPEED_RADPS * SwerveDef.TURN_COEFFICIENT; 

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
        double leftX = deadzone(controller.getLeftX());
        double leftY = deadzone(controller.getLeftY());
        double rightX = deadzone(controller.getRightX());

        xSpeed = leftX * leftX *getPositivity(leftX)* SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT + sideways * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        ySpeed = leftY * leftY *getPositivity(leftY)* SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT + forward * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        rotation = rightX * rightX *getPositivity(rightX)* SwerveDef.MAX_SPEED_RADPS * SwerveDef.TURN_COEFFICIENT;

        if(slowmode) {
            xSpeed /= 3;
            ySpeed /= 3;
            rotation /= 3;
        }
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rotation));
        //SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDef.MAX_SPEED_MPS);

        SwerveDef.flModule.setState(states[0]);
        SwerveDef.frModule.setState(states[1]);
        SwerveDef.rlModule.setState(states[2]);
        SwerveDef.rrModule.setState(states[3]);
    }

    

    public static void gyroMoverRamp(boolean condition){
        if(!condition) {
            forward = 0;
            sideways = 0;
        }
        
        if (Math.abs(SwerveDef.gyro.getRoll()) < 7 && Math.abs(SwerveDef.gyro.getPitch()) < 7) {
            forward = 0;
            sideways = 0;
            return;
        } 

        if(condition) {
            forward = -SwerveDef.gyro.getPitch();
            sideways = -SwerveDef.gyro.getRoll();
        }

    }

    static double getPositivity(double number){
        if (number == 0) return 0;
        return number > 0 ? 1 : -1;
    }

    /**
     * Function for setting module speeds based on controller input during field oriented driving
     */
    public static void orientedDrive() {
        //gyroMoverRamp(controller.getXButtonPressed());

        double leftX = deadzone(controller.getLeftX());
        double leftY = deadzone(controller.getLeftY());
        double rightX = deadzone(controller.getRightX());

        xSpeed = leftX * leftX *getPositivity(leftX)* SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT + sideways * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        ySpeed = leftY * leftY *getPositivity(leftY)* SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT + forward * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        rotation = rightX * rightX *getPositivity(rightX)* SwerveDef.MAX_SPEED_RADPS * SwerveDef.TURN_COEFFICIENT;

        if(slowmode) {
            xSpeed /= 3;
            ySpeed /= 3;
            rotation /= 3;
        }

        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, SwerveDef.gyro.getRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDef.MAX_SPEED_MPS);

        SwerveDef.flModule.setState(states[0]);
        SwerveDef.frModule.setState(states[1]);
        SwerveDef.rlModule.setState(states[2]);
        SwerveDef.rrModule.setState(states[3]);
    }

    /**public static void assistedDrive() {
        pid.setOffset(LimelightAiming.tapeLimelight1.X);

        ySpeed = deadzone(controller.getLeftY()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        xSpeed = pid.pidGet();
        rotation = deadzone(controller.getRightX()) * SwerveDef.MAX_SPEED_RADPS * SwerveDef.TURN_COEFFICIENT;

        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, SwerveDef.gyro.getRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDef.MAX_SPEED_MPS);

        SwerveDef.flModule.setState(states[0]);
        SwerveDef.frModule.setState(states[1]);
        SwerveDef.rlModule.setState(states[2]);
        SwerveDef.rrModule.setState(states[3]);
    }*/

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


    static boolean isGyroReset = false;

    /**
     * Function for zeroing gyroscope heading
     * Should not be used much during competition, takes long-ish time
     */
    static void gyroReset() {
        if(controller.getStartButton() && !isGyroReset){
            isGyroReset = true;
            SwerveDef.gyro.reset();

        }
        else{
            isGyroReset = false;
        }
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
        SmartDashboard.putNumber("FL encoder", SwerveDef.flModule.getNeoAngle());
        SmartDashboard.putNumber("FR encoder", SwerveDef.frModule.getNeoAngle());
        SmartDashboard.putNumber("RL encoder", SwerveDef.rlModule.getNeoAngle());
        SmartDashboard.putNumber("RR encoder", SwerveDef.rrModule.getNeoAngle());


        SmartDashboard.putNumber("gyro angle", SwerveDef.gyro.getAngle());

        SmartDashboard.putBoolean("auto aiming", assistedDrive);
        SmartDashboard.putBoolean("auto ramp move", rampToggle);
    }

    static class assistPID extends PIDController{
        double setpoint = 0;
        double maxSpeed = 0;
        double offset = 0;
        public assistPID(double kP, double kI, double kD, double setpoint) {
            super(kP, kI, kD);
            this.setpoint = setpoint;
            this.maxSpeed = SwerveDef.MAX_SPEED_MPS*SwerveDef.DRIVE_COEFFICIENT;
        }

        public void setOffset(double value) {
            offset = value;
        }

        public double pidGet() {
            double speed = MathUtil.clamp(super.calculate(offset, setpoint), -maxSpeed, maxSpeed);
            return -speed;
        }
    }
}
