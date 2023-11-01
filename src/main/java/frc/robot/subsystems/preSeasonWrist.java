package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntake;




public class preSeasonWrist extends SubsystemBase{
    private SparkMaxAbsoluteEncoder wristEncoder;
    private CANSparkMax wristMotor;
    private SparkMaxPIDController controller;
    double initialPos;

    public preSeasonWrist(){
        wristMotor = new CANSparkMax(ArmIntake.WRIST_SPARKMAX, MotorType.kBrushless);
        wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        wristEncoder.setPositionConversionFactor(360);
        initialPos = 269.33;
    }

    public double getDegrees(){
        return (wristEncoder.getPosition());
    }

    public void periodic() {
        SmartDashboard.putNumber("WristEncoderDegrees", getDegrees()-initialPos);
    }


}
