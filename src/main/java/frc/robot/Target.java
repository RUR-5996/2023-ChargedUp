package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Target {
    public boolean aligning = false;
    public boolean placing = false;


    public static void periodic() {
        if (!aligning && !placing && detect_target) {
            
        }

    }
        
    private boolean detect_target() {

    }

    private void align() {

    }

    private void place_cone(int step) {

    }

    private void place_block(int step) {

    }
}