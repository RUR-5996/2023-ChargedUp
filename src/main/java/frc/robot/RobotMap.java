package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class RobotMap {

    /*
     * Joystick map:
     * xbox:
     * left stick: drive x-y
     * right stick: rotate
     */

    // TODO maybe add PS controller instead?
    public static XboxController controller = new XboxController(0);

    public static XboxController secondController = new XboxController(1);

    public static final WPI_VictorSPX mover1 = new WPI_VictorSPX(0);
    public static final WPI_VictorSPX mover2 = new WPI_VictorSPX(1);
    public static final WPI_VictorSPX gripper = new WPI_VictorSPX(2);
}
