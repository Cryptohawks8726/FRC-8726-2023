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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class preSeasonArm extends SubsystemBase {
    private SparkMaxAbsoluteEncoder encoder;
    private CANSparkMax motor;
    private ProfiledPIDController pidController;
    private ArmFeedforward armFF;
    private Constraints contraints;
    

    public preSeasonArm() {
        motor = new CANSparkMax(Arm.ARM_SPARKMAX, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(2*Math.PI);
        contraints = new Constraints(.5f, 1f);
        pidController = new ProfiledPIDController(2f, 0f, .25f, contraints); //new ProfiledPIDController(2f,0f,.25f);
        pidController.setGoal(-1.57f); // resting
        armFF = new ArmFeedforward(Arm.ARM_kS,Arm.ARM_kG,Arm.ARM_kV);
    }

    @Override
    public void periodic() {
        //encoder.setPositionConversionFactor(360);
        //SmartDashboard.putNumber("ArmEncoderPos",encoder.getPosition());
        double currentOutput = pidController.calculate(getRadians());

        
        //pidController.setSetpoint(0);
        // feed foward WARNING: GET VELOCITY ERROR MIGHT NOT BE SAME AS EXACT VELOCITY
        // pidController.getVelocityError()
        double ff = armFF.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
        SmartDashboard.putNumber("actualposition", encoder.getPosition());
        SmartDashboard.putNumber("velocity", pidController.getSetpoint().velocity);
        SmartDashboard.putNumber("position", pidController.getSetpoint().position);
        //motor.setVoltage(ff+currentOutput);

    }

    public void setGoal(double radians) {
        pidController.setGoal(radians);
    }

    public double getRadians(){
        return encoder.getPosition() - 3.825 - (Math.PI/2); // - Arm.ENCODER_OFFSET_SUBTRACT*Math.PI/180
    }
}
