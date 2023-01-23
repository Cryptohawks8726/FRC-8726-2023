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

public class HandSubsystem extends SubsystemBase {
  private CANSparkMax handMotor;
  private SparkMaxPIDController pid;
  private RelativeEncoder handEncoder;

  public HandSubsystem() {
    handMotor = new CANSparkMax(Constants.HAND_MOTOR_SPARKMAX, MotorType.kBrushless);
    handEncoder = handMotor.getEncoder();
    pid = handMotor.getPIDController(); //The three parameters are the proportional term (position error to 0), the derivative term (velocity error to 0), and the integral term (total accumulated error over time to 0)

    handMotor.setIdleMode(IdleMode.kBrake);
    // handMotor.setSmartCurrentLimit(20,20);

    pid.setP(Constants.HAND_kP);
    pid.setD(Constants.HAND_kI);
    pid.setI(Constants.HAND_kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Constants.handCurr.setDouble(handMotor.getOutputCurrent()); // gets the velocity instead of current supply (cant find the function)
  }

  public RelativeEncoder getHandEncoder() {
    return handEncoder;
  }

  public void setHandPower(double speed){
    pid.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

}
