package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Manipulator {
    //dva falcony zvedaj, redline nabírá

    enum Move {
        NONE,
        MOVEUP,
        MOVEDOWN,
    }
    
    enum Grab {
        HOLD,
        GRIP,
        LETGO,
    }

    static Timer buttonTimer = new Timer();

    static Move move = Move.NONE;
    static Grab grab = Grab.HOLD;

    static boolean hasButtonStopped = false;

    public static void periodic() {
        moveCheck();
        grabCheck();
    }

    public static void toggleMove(Move moveToToggle, boolean condition) {
        if (move == moveToToggle)
            move = Move.NONE;
        else
            move = moveToToggle;
    }

    public static void toggleGrab(Grab grabToToggle, boolean condition) {
        if (grab == grabToToggle)
            grab = Grab.HOLD;
        else
            grab = grabToToggle;
    }

    public static void grabCheck() {
        double gripArmMovement = 0;

        toggleGrab(Grab.GRIP, RobotMap.controller.getYButtonPressed());
        toggleGrab(Grab.LETGO, RobotMap.controller.getXButtonPressed());

        switch (grab) {
            case HOLD:
                gripArmMovement = 0;
                break;
            case GRIP:
                gripArmMovement = Constants.GRAB_SPEED * -1;
                break;
            case LETGO:
                gripArmMovement = Constants.GRAB_SPEED;
                break;
        }

        RobotMap.gripper.set(gripArmMovement);
    }

    public static void moveCheck() {
        double gripperMovement = 0;

        toggleMove(Move.MOVEUP, RobotMap.controller.getYButtonPressed());
        toggleMove(Move.MOVEDOWN, RobotMap.controller.getXButtonPressed());

        switch (move) {
            case NONE:
                gripperMovement = 0;
                break;
            case MOVEUP:
                gripperMovement = Constants.ARM_MOVEMENT_SPEED;
                break;
            case MOVEDOWN:
                gripperMovement = Constants.ARM_MOVEMENT_SPEED * -1;
                break;

        }

        RobotMap.mover1.set(gripperMovement);
        RobotMap.mover2.set(gripperMovement);
    }

}
