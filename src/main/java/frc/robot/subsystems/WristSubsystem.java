// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;


public class WristSubsystem extends SubsystemBase {
  private DutyCycleEncoder encoder;
  private double encoderVal;
  private Timer timer;
  private boolean flag;
  private CANSparkMax wristMotor;

  public WristSubsystem() {
    timer = new Timer();
    timer.stop();
    timer.reset();
    timer.start();

    encoder = new DutyCycleEncoder(Constants.WRIST_ENCODER_CHANNEL);

    flag = true;

    wristMotor = new CANSparkMax(Constants.WRIST_MOTOR_SPARKMAX, MotorType.kBrushed);
    wristMotor.setIdleMode(IdleMode.kCoast);
    }

  @Override
  public void periodic() {
    if(timer.get() > Constants.ENCODER_TIME_DELAY) {
      if(flag) {
        flag = false;
        encoder.reset();
        encoder.setPositionOffset(encoder.getAbsolutePosition());
      }
      encoderVal = encoder.getAbsolutePosition() - encoder.getPositionOffset();
    }
  }

  public double getEncoderPos() {
    return encoderVal;
  }

  public void raiseArm() {
    wristMotor.set(Constants.RAISE_WRIST_SPEED);
  }

  public void lowerArm() {
    wristMotor.set(Constants.LOWER_WRIST_SPEED);
  }

  public void kill() {
    wristMotor.set(0.0);
  }
}
