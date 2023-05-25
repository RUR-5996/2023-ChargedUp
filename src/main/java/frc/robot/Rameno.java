package frc.robot; //TODO cleanup, move functions and variables around to make more sense

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Rameno {

    static double FALCON_TICKS_PER_REV = 2048.0;
    static double ARM_GEAR_RATIO = 0.009375;

    static TalonFXInvertType mover1Inverted = TalonFXInvertType.CounterClockwise;
    static TalonFXInvertType mover2Inverted = TalonFXInvertType.Clockwise;
    static double feedbackCoefficient = (FALCON_TICKS_PER_REV * ARM_GEAR_RATIO); //counts in arm degrees TODO check the scale

    static Position[] positions = new Position[6]; //TODO make positions for all the scenarios + name them for clear interpretation
    static int currentPositionId = 1;
    static boolean engaged = false;

    static Timer releaseTimer = new Timer();
    static boolean releasing = false;

    static XboxController sController = RobotMap.secondController;
    static boolean triggered = false;

    public static void motorInit() {
        RobotMap.mover1.configFactoryDefault();
        RobotMap.mover1.setInverted(mover1Inverted);
        RobotMap.mover1.configOpenloopRamp(0);
        RobotMap.mover1.configClosedloopRamp(0); //try this to save up the gears
        RobotMap.mover1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        RobotMap.mover1.setSelectedSensorPosition(0);
        RobotMap.mover1.configSelectedFeedbackCoefficient(1/feedbackCoefficient);
        RobotMap.mover1.config_kP(0, 0.4);
        RobotMap.mover1.config_kI(0, 0);
        RobotMap.mover1.config_kD(0, 0);

        RobotMap.mover2.configFactoryDefault();
        RobotMap.mover2.setInverted(mover2Inverted);
        RobotMap.mover2.configOpenloopRamp(0);
        RobotMap.mover2.configClosedloopRamp(0);
        RobotMap.mover2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        RobotMap.mover2.configSelectedFeedbackCoefficient(1/feedbackCoefficient);
        RobotMap.mover2.setSelectedSensorPosition(0);
        RobotMap.mover2.config_kP(0, 0.5);
        RobotMap.mover2.config_kI(0, 0);
        RobotMap.mover2.config_kD(0, 0);
        RobotMap.mover2.follow(RobotMap.mover1);

        RobotMap.release.configFactoryDefault();
        RobotMap.release.setInverted(false); //use constant
        RobotMap.release.configOpenloopRamp(0.2);
        RobotMap.release.configClosedloopRamp(0.2);
    }

    public static void teleopInit() {
        RobotMap.mover1.setNeutralMode(NeutralMode.Brake);
        RobotMap.mover2.setNeutralMode(NeutralMode.Brake);
    }

    public static void disabledInit() {
        RobotMap.mover1.setNeutralMode(NeutralMode.Coast);
        RobotMap.mover2.setNeutralMode(NeutralMode.Coast);
        RobotMap.release.setNeutralMode(NeutralMode.Coast);
        
    }

    public static void robotInit() {
        setPositions();
        motorInit();
    }

    public static void disabledPeriodic() {
        if(sController.getAButton()) {
            RobotMap.mover1.setSelectedSensorPosition(0);
        }
    }

    static final double ENCODER_POSITION_COEF = 4; // should be 4 if 3 is working

    public static void setPositions() { //TODO make widget and name positions
        positions[0] = new Position(400 * ENCODER_POSITION_COEF, 0); //lowest position
        positions[1] = new Position(800* ENCODER_POSITION_COEF, 1);
        positions[2] = new Position(1100* ENCODER_POSITION_COEF, 2);
        positions[3] = new Position(1700* ENCODER_POSITION_COEF, 3);
        positions[4] = new Position(2850 * ENCODER_POSITION_COEF, 4);
        positions[5] = new Position(3200* ENCODER_POSITION_COEF, 5);
    }

    public static void releaseArm() {
        if(releasing && releaseTimer.get() < 1) {
            RobotMap.release.set(0.75);
        } else if (releasing && releaseTimer.get() > 1) {
            releasing = false;
            RobotMap.release.set(0); //TODO check logic
        }
    }

    public static void tryStartRelease() {
        if(sController.getBackButtonPressed()) { //TODO bind this in autonomous to release at the start of game
            startRelease();
        }
    }

    public static void startRelease(){
        releasing = true;
        releaseTimer.reset();
        releaseTimer.start();
    }

    

    public static void setPosition() {
        RobotMap.mover1.set(ControlMode.Position, positions[currentPositionId].getValue() + adder); //the other motor should follow
    }

    public static void changePositionAutonomous(int position){ //TODO check this
        currentPositionId = position;
        triggered = false;
    }
    static double adder = 5;
    public static void changePosition() {
        if(sController.getPOV() == 0 && !triggered && currentPositionId < positions.length - 1) {
            currentPositionId += 1;
            triggered = true;
        } else if (sController.getPOV() == 180 && !triggered && currentPositionId > 0) {
            currentPositionId -= 1;
            triggered = true;
        
        } 
        if (sController.getYButtonReleased()) {
            adder += 100 * ENCODER_POSITION_COEF;
        } else if (sController.getXButtonReleased()) {
            adder -= 100 * ENCODER_POSITION_COEF;
        }

        if(sController.getPOV() == -1) {
            triggered = false;
        }
    }

    public static void disengageArm() {
        RobotMap.mover1.set(ControlMode.PercentOutput, 0); //disengage PID control, protect the motors and battery
    }

    public static void setEngagement() {
        if(sController.getAButtonPressed()) { //TODO check functionality, might need correction
            engaged = !engaged;
        }
    }

    public static void teleopPeriodic() {
        if(releasing) {
            releaseArm();
            disengageArm();
        } else if (engaged) {
            setPosition(); //TODO test engaging without moving
        } else {
            disengageArm();
        }

        changePosition();
        tryStartRelease(); //TODO make this autonomous for comp, maybe allow this to happen once
        setEngagement();
    }

    public static void autonomousPeriodic(){
        if(releasing) {
            releaseArm();
            disengageArm();
        } else if (engaged) {
            setPosition(); 
        } else {
            disengageArm();
        }

        
    }

    public static void report() {
        SmartDashboard.putNumber("current arm position", currentPositionId); //TODO should be done through Shuffleboard next time - can be tested on testing board
        SmartDashboard.putNumber("arm current left", RobotMap.mover1.getStatorCurrent());
        SmartDashboard.putNumber("arm current right", RobotMap.mover2.getStatorCurrent());
        SmartDashboard.putNumber("arm left position", RobotMap.mover1.getSelectedSensorPosition());
        SmartDashboard.putNumber("arm right position", RobotMap.mover2.getSelectedSensorPosition());

        SmartDashboard.putBoolean("pidEngaged", engaged);
    }

    static class Position { //easy for storing info about each position
        double encoderValue;
        double order;
        String alternateName;

        public Position(double encoderValue, double order) {
            this.encoderValue = encoderValue;
            this.order = order;
            this.alternateName = "";
        }

        public Position(double encoderValue, double order, String alternateString) {
            this.encoderValue = encoderValue;
            this.order = order;
            this.alternateName = alternateString; //for future Shuffleboard widget
        }

        public double getOrder() {
            return order;
        }

        public double getValue() {
            return encoderValue;
        }

        public String getName() {
            return alternateName;
        }
    }
}
