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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class preSeasonArm extends SubsystemBase {
    private SparkMaxAbsoluteEncoder encoder;
    private CANSparkMax motor;
    private PIDController pidController;
    

    public preSeasonArm() {
        motor = new CANSparkMax(Arm.ARM_SPARKMAX, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        pidController = new PIDController(10f,0f,0f);
    }

    @Override
    public void periodic() {
        encoder.setPositionConversionFactor(360);
        SmartDashboard.putNumber("ArmEncoderPos",encoder.getPosition());
    }
}
