// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;



public class ArmSubsystem extends SubsystemBase {
  private DutyCycleEncoder encoder;
  private double encoderVal;
  private Timer timer;
  private boolean flag;
  private PIDController posPID;
  private SparkMaxPIDController velPID;
  private CANSparkMax armMotor;

  public ArmSubsystem() {
    timer = new Timer();
    timer.stop();
    timer.reset();
    timer.start();

    encoder = new DutyCycleEncoder(3);

    flag = true;

    armMotor = new CANSparkMax(Constants.ARM_MOTOR_SPARKMAX, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kCoast);

    posPID = new PIDController(Constants.ARM_kP, Constants.ARM_kI, Constants.ARM_kD);
    velPID = armMotor.getPIDController();
    velPID.setP(1);
  }

  @Override
  public void periodic() {
    if(timer.get() > 3) {
      if(flag) {
        flag = false;
        encoder.reset();
        encoder.setPositionOffset(encoder.getAbsolutePosition());
      }
      encoderVal = encoder.getAbsolutePosition() - encoder.getPositionOffset();
    }
  }

  public void setRefPoint(double pos) {
    posPID.setSetpoint(pos);
  }

  public double getEncoderPos() {
    return encoderVal;
  }

  public void raiseArm() {
    // velPID.setReference(10, ControlType.kVelocity);
    // System.out.println("raising");
    armMotor.set(Constants.RAISE_ARM_SPEED);
  }

  public void lowerArm() {
    // velPID.setReference(-10, ControlType.kVelocity);
    // System.out.println("lowering");
    armMotor.set(Constants.LOWER_ARM_SPEED);
  }

  public void coneHeld() {
    posPID.setP(Constants.ARM_kP);
    posPID.setI(Constants.ARM_kI);
    posPID.setD(Constants.ARM_kD);
  }

  public void coneNotHeld() {
    posPID.setP(Constants.CONEHELD_kP);
    posPID.setI(Constants.CONEHELD_kI);
    posPID.setD(Constants.CONEHELD_kD);
  }

  public void kill() {
    armMotor.set(0.0);
  }

  public void stay() {
    armMotor.set(-posPID.calculate(encoderVal));
  }
}
