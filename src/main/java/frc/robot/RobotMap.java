package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class RobotMap {

    /*
     * Joystick map:
     * xbox:
     * left stick: drive x-y
     * right stick: rotate
     */

    //TODO maybe add PS controller instead?
    public static XboxController controller = new XboxController(0);

    public static final WPI_TalonFX mover1 = new WPI_TalonFX(0);
    public static final WPI_TalonFX mover2 = new WPI_TalonFX(1);
    public static final WPI_VictorSPX gripper = new WPI_VictorSPX(2);

    //TODO address joystick usage
    public static Joystick logitech = new Joystick(1);

    public static Trigger logitechTrigger = new JoystickButton(logitech, 1);
    public static final Trigger logitechTwo = new JoystickButton(logitech, 2);
    public static final Trigger logitechThree = new JoystickButton(logitech, 3);
    public static final Trigger logitechFour = new JoystickButton(logitech, 4);
    public static final Trigger logitechFive = new JoystickButton(logitech, 5);
    public static final Trigger logitechSix = new JoystickButton(logitech, 6);
    public static final Trigger logitechSeven = new JoystickButton(logitech, 7);
    public static final Trigger logitechEight = new JoystickButton(logitech, 8);
    public static final Trigger logitechNine = new JoystickButton(logitech, 9);
    public static final Trigger logitechTen = new JoystickButton(logitech, 10);
    public static final Trigger logitechEleven = new JoystickButton(logitech, 11);

}