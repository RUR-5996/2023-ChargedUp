package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sensors {

    // Limelight 
    static enum LimelightMode {
        PROCESSING,
        DRIVING
    }

    static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight"); // might need to change table
                                                                                         // name
    static NetworkTableEntry limelightTx = limelightTable.getEntry("tx");
    static NetworkTableEntry limelightTy = limelightTable.getEntry("ty");
    static NetworkTableEntry limelightTa = limelightTable.getEntry("ta");
    public static NetworkTableEntry limelightPipeline = limelightTable.getEntry("pipeline");

    static double limelightX = 0.0;
    static double limelightY = 0.0;
    static double limelightA = 0.0;

    public static LimelightMode currentMode = LimelightMode.DRIVING;

    // PUBLIC

    public static void periodic(){
        runLimelight();

        if (RobotMap.controller.getBButtonPressed()) {
            switchLimelightMode();
        }

        report();
    }

    public static void report() {
        SmartDashboard.putString("LimelightMode", currentMode.toString());
        SmartDashboard.putNumber("LimelightX", limelightX);
        SmartDashboard.putNumber("LimelightY", limelightY);
        SmartDashboard.putNumber("LimelightArea", limelightA);   
    }

    public static void switchLimelightMode() {
        switch (currentMode) {
            case PROCESSING:
                currentMode = LimelightMode.DRIVING;
                limelightPipeline.setNumber(1); // need to configure this on the camera itself
                break;
            case DRIVING:
                currentMode = LimelightMode.PROCESSING;
                limelightPipeline.setNumber(0); // 
                break;
            default:
                limelightPipeline.setNumber(1);
                currentMode = LimelightMode.DRIVING;
                break;
        }
    }

    // PRIVATE

    static void runLimelight(){
        limelightX = limelightTx.getDouble(0.0);
        limelightY = limelightTy.getDouble(0.0);
        limelightA = limelightTa.getDouble(0.0);
    }
}
