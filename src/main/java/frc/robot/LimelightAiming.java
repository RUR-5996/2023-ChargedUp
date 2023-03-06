package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveDef.pidValues;


class LimelightNetwork{
    NetworkTable limelightTable; // might need to change table

    NetworkTableEntry tX;
    NetworkTableEntry tY;
    NetworkTableEntry tA;
    NetworkTableEntry tV;
    public NetworkTableEntry pipeline;

    public double X = 0.0;
    public double Y = 0.0;
    public double A = 0.0;
    public boolean V = false;

    public LimelightNetwork(String key){
        limelightTable = NetworkTableInstance.getDefault().getTable(key);

        tX = limelightTable.getEntry("tx");
        tY = limelightTable.getEntry("ty");
        tA = limelightTable.getEntry("ta");
        tV = limelightTable.getEntry("tv");
        pipeline = limelightTable.getEntry("pipeline");
    }

    public void update(){
        X = tX.getDouble(0.0);
        Y = tY.getDouble(0.0);
        A = tA.getDouble(0.0);
        V = tV.getDouble(0.0) == 1.0;
    }
}


public class LimelightAiming{
    
    static LimelightNetwork tapeLimelight1;
    static LimelightNetwork tapeLimelight2;


    // Limelight 
    static enum LimelightMode {
        PROCESSING,
        DRIVING
    }

    public static final double MIN_TAPE_DISTANCE_AREA = 1;
    public static final double ROTATION_COEFICIENT = 0.4;
    public static final double MINIMAL_ROTATION = 3;

    public static boolean canAim = false;
    

    // PUBLIC

    public static void init(){
        tapeLimelight1 = new LimelightNetwork("limelight1"); //tries to find 2 different cameras
        tapeLimelight2 = new LimelightNetwork("limelight2");
    }

    public static void periodic(){
        runLimelight();
        SwerveDrive.limelightAimRotation = 0;

        if (RobotMap.controller.getBButtonPressed()) {
            canAim = !canAim;
        }

        aim();


        report();
    }
    static void aim(){
        if(!canAim) return; //watch the return in void functions!
        
        LimelightNetwork correctLimelight;

        if(tapeLimelight1.V && tapeLimelight2.V){ //would require quick switching between pipelines - hard to achieve

            if(tapeLimelight1.Y > tapeLimelight2.Y) 
                correctLimelight = tapeLimelight1;
            else 
                correctLimelight = tapeLimelight2;
        }
        else if(tapeLimelight1.V){
            correctLimelight = tapeLimelight1;
        }
        else if(tapeLimelight2.V){
            correctLimelight = tapeLimelight2;
        }
        else{
            return;
        }
        //good idea, but would require 2 cameras for maximum effectivity

        double rightAngle = -correctLimelight.X;

        if(MIN_TAPE_DISTANCE_AREA > correctLimelight.A) return;
        if(rightAngle <  MINIMAL_ROTATION) return;
        
        SwerveDrive.limelightAimRotation = rightAngle * ROTATION_COEFICIENT; //is rotation enough? we could possibly be angled correctly but be off to one side. Alternatively, if we turn too much, we could hit the divider
        //how do we actually use this?
    }

    public static void report() {
        /* 
        SmartDashboard.putString("LimelightMode", currentMode.toString());
        SmartDashboard.putNumber("LimelightX", limelightX);
        SmartDashboard.putNumber("LimelightY", limelightY);
        SmartDashboard.putNumber("LimelightArea", limelightA);   */
    }

    

    // PRIVATE

    static void runLimelight(){
        tapeLimelight1.update();
        tapeLimelight2.update();
    }
}
