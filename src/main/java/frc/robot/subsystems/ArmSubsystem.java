// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends SubsystemBase {
  private SparkMaxAbsoluteEncoder encoder;
  private double encoderVal;
  private Timer timer;
  private boolean flag = true;
  private boolean isConeHeld;
  private SparkMaxPIDController posPID;
  private CANSparkMax armMotor;

  public ArmSubsystem() {
    timer = new Timer();
    timer.reset();
    timer.start();
    isConeHeld = false;
    //encoder = new DutyCycleEncoder(Constants.ARM_ENCODER_CHANNEL); // if encoder is wired to rio

    armMotor = new CANSparkMax(Constants.ARM_MOTOR_SPARKMAX, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kCoast);

    encoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    //posPID = new PIDController(Constants.ARM_kP, Constants.ARM_kI, Constants.ARM_kD);
    posPID = armMotor.getPIDController();
    posPID.setP(Constants.ARM_kP);
    posPID.setI(Constants.ARM_kI);
    posPID.setD(Constants.ARM_kD);
  }


  @Override
  public void periodic() {
    if (flag){
      zeroOffset();
      // zeroEncoder();
      flag = false;
    }
    SmartDashboard.putNumber("Arm Pos",encoder.getPosition());
  }

  public void setPosRefPoint(double pos) {
    posPID.setReference(pos, ControlType.kPosition);
  }

  public void setVelRefPoint(double vel){
    posPID.setReference(vel, ControlType.kVelocity);
  }
  
  public double getEncoderPos() {
    return encoderVal;
  }

  public void raiseArm() {
    setVelRefPoint(Constants.RAISE_ARM_SPEED);
    //armMotor.set(Constants.RAISE_ARM_SPEED);
  }

  public void lowerArm() {
    setVelRefPoint(Constants.LOWER_ARM_SPEED);
    //armMotor.set(Constants.LOWER_ARM_SPEED);
  }

  
  public void coneNotHeld() {
    isConeHeld = true;
    posPID.setP(Constants.ARM_kP);
    posPID.setI(Constants.ARM_kI);
    posPID.setD(Constants.ARM_kD);
  }

  public void coneHeld() {
    isConeHeld = false;
    posPID.setP(Constants.CONEHELD_kP);
    posPID.setI(Constants.CONEHELD_kI);
    posPID.setD(Constants.CONEHELD_kD);
  }

  public void stay() {
    setPosRefPoint(encoder.getPosition());
    //armMotor.set(-posPID.calculate(encoderVal)); // why is this negative? we can use the 
  }

  public void zeroEncoder(){ // remove once we have offsets. I don't see a use for generating new offsets every match
    encoder.setZeroOffset(encoder.getPosition()+encoder.getZeroOffset());
    SmartDashboard.putNumber("Arm Encoder Offset",encoder.getZeroOffset());
    SmartDashboard.putNumber("Arm Pos after Offset",encoder.getPosition()); // should be 0, double checking
  }

  public void zeroOffset(){
    encoder.setZeroOffset(0.0);
  }
}
