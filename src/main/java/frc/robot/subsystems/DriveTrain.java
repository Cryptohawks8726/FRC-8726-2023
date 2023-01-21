package frc.robot.subsystems;
import  com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain {

    private CANSparkMax leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor;
    
    public DriveTrain() {
        
        leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_MOTOR_SPARKMAX, MotorType.kBrushless);
        leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR_SPARKMAX, MotorType.kBrushless);
        rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_MOTOR_SPARKMAX, MotorType.kBrushless);
        rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR_SPARKMAX, MotorType.kBrushless);

    }

}
