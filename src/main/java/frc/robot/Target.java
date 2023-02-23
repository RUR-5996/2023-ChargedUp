package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Target {
    static boolean aligning = false;
    static boolean placing = false;
    static boolean vibrated = false;
    static boolean placing_cone; //false if placing cube
    static int step;


    public static void periodic() {
        if (!aligning && !placing && detect_target()) {
            if(!vibrated){
                //vibrate function
                vibrated = true;
            }
        }
        else {
            vibrated = false;

            if(aligning) {
                align();
            }
            else if(placing && placing_cone) {
                if(place_cone(step))
            }
            else if(placing) {
                place_cube(step);
            }
        }


    }
        
    private boolean detect_target() {
        return (Sensors.NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) ? true : false;
    }

    private boolean align() { //periodic function for aligning, returns true when it's done
        
    }

    private boolean place_cone(int level) { //periodic function for placing a cone, takes the level (0-2) where it will place the cone, returns true when it's done. Needs to be programmed with manually measured steps

    }

    private boolean place_block(int level) { //periodic function for placing a cube, takes the level (0-2) where it will place the cube, returns true when it's done. Needs to be programmed with manually measured steps

    }
}