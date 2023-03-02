package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.XboxController;

public class Manipulator {
    //dva falcony zvedaj, redline nabírá

    static XboxController secondController = RobotMap.secondController;

    enum Move {
        NONE,
        MOVE,
/*        MOVEUP,
          MOVEDOWN,
*/        
    }
    
    enum Grab {
        HOLD,
        GRIP,
        LIGHTGRIP,
        LETGO,
    }

    static Timer buttonTimer = new Timer();

    static long startTime;
    static boolean isGripping = false;

    static Move move = Move.NONE;
    static Grab grab = Grab.HOLD;

    static boolean hasButtonStopped = false;

    static boolean intakeIsMooving = false;

    static boolean mover1Inverted = false;
    static boolean mover2Inverted = false;

    static double bottomPosition = 0;
    static double moverOffset = 0;

    public static void init() {
        RobotMap.mover1.setInverted(mover1Inverted);
        RobotMap.mover1.setNeutralMode(NeutralMode.Brake);
        RobotMap.mover1.configOpenloopRamp(0.2);
        RobotMap.mover1.setSelectedSensorPosition(0);
        RobotMap.mover1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);

        RobotMap.mover2.setInverted(mover2Inverted);
        RobotMap.mover2.setNeutralMode(NeutralMode.Brake);
        RobotMap.mover2.configOpenloopRamp(0.2);
        RobotMap.mover2.follow(RobotMap.mover1);
        RobotMap.mover2.setSelectedSensorPosition(0);
        RobotMap.mover2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
    }

    public static void disabledInit() {
        RobotMap.mover1.setNeutralMode(NeutralMode.Coast);
        RobotMap.mover2.setNeutralMode(NeutralMode.Coast);
    }

    public static void periodic() {
        moveCheck();
        grabCheck();
        lightGripSet();
        leftAxisUpdate();
    }

    public static void toggleMove(Move moveToToggle, boolean condition) {
        if (!condition)
            return;

        if (move == moveToToggle)
            move = Move.NONE;
        else
            move = moveToToggle;
    }

    public static void toggleGrab(Grab grabToToggle, boolean condition) {
        if (!condition)
            return;
        
        if (grab == grabToToggle)
            grab = Grab.HOLD;
        else
            grab = grabToToggle;
    }

    public static void grabCheck() {
        double gripArmMovement = 0;

        toggleGrab(Grab.GRIP, secondController.getBButtonPressed());
        toggleGrab(Grab.LETGO, secondController.getAButtonPressed());

        switch (grab) {
            case HOLD:
                gripArmMovement = 0;
                break;
            case GRIP:
                gripArmMovement = Constants.GRAB_SPEED * -1;
                startTime = System.currentTimeMillis();
                isGripping = true;
                break;
            case LETGO:
                gripArmMovement = Constants.GRAB_SPEED;
                break;
            case LIGHTGRIP:
                gripArmMovement = Constants.GRAB_SPEED * -0.05;
        }

        RobotMap.gripper.set(gripArmMovement);
    }


    
    public static void lightGripSet() {
        if (!isGripping) return;

        long currentTime = System.currentTimeMillis();
        if (currentTime - startTime > 100)
            toggleGrab(Grab.LIGHTGRIP, true);
        isGripping = false;
    }

    public static void leftAxisUpdate() {
        if (Math.abs(secondController.getLeftY()) < 0.1f)
            intakeIsMooving = false;
        else
            intakeIsMooving = true;
    }

    public static void moveCheck() {
        double gripperMovement = 0;

        toggleMove(Move.MOVE, intakeIsMooving);
        //toggleMove(Move.NONE, !intakeIsMooving);

//        toggleMove(Move.MOVEUP, secondController.getYButtonPressed());
//        toggleMove(Move.MOVEDOWN, secondController.getXButtonPressed());

        switch (move) {
            case NONE:
                gripperMovement = 0;
                break;
            case MOVE:
                gripperMovement = Constants.ARM_MOVEMENT_SPEED * secondController.getLeftY();
                break;
/*            case MOVEUP:
                gripperMovement = Constants.ARM_MOVEMENT_SPEED * 1;
                break;
            case MOVEDOWN:
                gripperMovement = Constants.ARM_MOVEMENT_SPEED * -1;
                break;
*/
        }

        RobotMap.mover1.set(ControlMode.Position, bottomPosition);
        RobotMap.mover2.set(ControlMode.Position, bottomPosition + moverOffset);

        moverOffset = RobotMap.mover1.getSelectedSensorPosition() - RobotMap.mover2.getSelectedSensorPosition();
    }

}
