package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {
    
    public static XboxController controller = new XboxController(0);
    public static XboxController secondController = new XboxController(1);


    public static CANSparkMax hoodMotor = new CANSparkMax(10, MotorType.kBrushless);
    public static RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    public static CANSparkMax turretMotor = new CANSparkMax(11, MotorType.kBrushless);
    public static RelativeEncoder turretEncoder = turretMotor.getEncoder();

    public static VictorSPX lowerShooter = new VictorSPX(5);
    public static VictorSPX upperShooter = new VictorSPX(4);

    public static VictorSPX leftIntake = new VictorSPX(7);
    public static VictorSPX rightIntake = new VictorSPX(8);

    public static VictorSPX intakeMotor = new VictorSPX(6);

    public static VictorSPX feederMotor = new VictorSPX(9);
}
