package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Test {
    
    static double coeff = 3;
    static double speed1 = 0.18;
    static double speed2 = 0.2;
    static double antiGravity = 1;
    static boolean mover1Inverted = true;
    static boolean mover2Inverted = false;

    public static void init() {
        RobotMap.mover1.setInverted(mover1Inverted);
        RobotMap.mover1.setNeutralMode(NeutralMode.Brake);
        RobotMap.mover1.configOpenloopRamp(0);
        RobotMap.mover1.configClosedloopRamp(0);

        RobotMap.mover2.setInverted(mover2Inverted);
        RobotMap.mover2.setNeutralMode(NeutralMode.Brake);
        RobotMap.mover2.configOpenloopRamp(0);
        RobotMap.mover2.configClosedloopRamp(0);
        //RobotMap.mover2.follow(RobotMap.mover1);
    }

    public static void disabledInit() {
        RobotMap.mover1.setNeutralMode(NeutralMode.Coast);
        RobotMap.mover2.setNeutralMode(NeutralMode.Coast);
    }

    public static void periodic() {
        if(RobotMap.secondController.getLeftY() > 0.5) {
            RobotMap.mover1.set(ControlMode.PercentOutput, speed1*coeff);
            RobotMap.mover2.set(ControlMode.PercentOutput, speed2*coeff);
        } else {
            RobotMap.mover1.set(ControlMode.PercentOutput, speed1*antiGravity);
            RobotMap.mover2.set(ControlMode.PercentOutput, speed2*antiGravity);
        }
        
    }

}
