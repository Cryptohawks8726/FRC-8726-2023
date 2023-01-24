// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private SparkMaxPIDController pid;
  private RelativeEncoder armEncoder;

  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_SPARKMAX, MotorType.kBrushless);
    // armEncoder = armMotor.getEncoder();
    // pid = armMotor.getPIDController(); //The three parameters are the proportional term (position error to 0), the derivative term (velocity error to 0), and the integral term (total accumulated error over time to 0)

    armMotor.setIdleMode(IdleMode.kBrake);
    // armMotor.setSmartCurrentLimit(20,20);

    // pid.setP(Constants.ARM_kP);
    // pid.setD(Constants.ARM_kD);
    // pid.setI(Constants.ARM_kI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Constants.armCurr.setDouble(armMotor.getOutputCurrent()); // gets the velocity instead of current supply (cant find the function)
  }

  public RelativeEncoder getArmEncoder() {
    return armEncoder;
  }

  public void setArmPower(double speed){
    // pid.setReference(speed, CANSparkMax.ControlType.kVelocity);
    armMotor.set(speed);
  }

}
