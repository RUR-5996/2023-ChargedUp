package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

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

    public static final WPI_TalonFX mover1 = new WPI_TalonFX(7);
    public static final WPI_TalonFX mover2 = new WPI_TalonFX(8); //right
    public static final WPI_VictorSPX gripper = new WPI_VictorSPX(2);
    public static final WPI_VictorSPX release = new WPI_VictorSPX(3);
    public static final AHRS gyro = new AHRS(SPI.Port.kMXP);
}
