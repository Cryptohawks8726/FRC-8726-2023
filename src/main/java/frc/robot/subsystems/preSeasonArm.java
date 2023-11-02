// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.Arm;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class preSeasonArm extends SubsystemBase {
    private SparkMaxAbsoluteEncoder encoder;
    private CANSparkMax motor;
    private PIDController pidController;
    private ArmFeedforward armFF;
    

    public preSeasonArm() {
        motor = new CANSparkMax(Arm.ARM_SPARKMAX, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        pidController = new PIDController(0.1f,0.03f,0f);
        armFF = new ArmFeedforward(Arm.ARM_kS,Arm.ARM_kG,Arm.ARM_kV);
    }

    @Override
    public void periodic() {
        //encoder.setPositionConversionFactor(360);
        //SmartDashboard.putNumber("ArmEncoderPos",encoder.getPosition());
        double currentOutput = pidController.calculate(getRadians());
        // feed foward
        double ff = armFF.calculate(pidController.getSetpoint(), pidController.getVelocityError());
        motor.setVoltage(currentOutput+ff);
    }

    public void setGoal() {

    }

    public double getDegrees(){
        return encoder.getPosition() - Arm.ENCODER_OFFSET_SUBTRACT;
    }
    
    public double getRadians(){
        return Math.toRadians(getDegrees());
    }
}
