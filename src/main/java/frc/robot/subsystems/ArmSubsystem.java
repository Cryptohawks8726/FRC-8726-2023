// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.SparkMaxPIDController;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private PIDController armPID;
  private SparkMaxAbsoluteEncoder armEncoder;
  

  public ArmSubsystem() {

    
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_SPARKMAX, MotorType.kBrushless);
    armPID = new PIDController(Constants.ARM_kP, Constants.ARM_kI, Constants.ARM_kD);
    armEncoder=armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
    armMotor.setIdleMode(IdleMode.kBrake);
    

    
    //pid = armMotor.getPIDController(); //The three parameters are the proportional term (position error to 0), the derivative term (velocity error to 0), and the integral term (total accumulated error over time to 0)
    // armMotor.setSmartCurrentLimit(20,20);

    armMotor.setIdleMode(IdleMode.kBrake);

    
  
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Constants.armCurr.setDouble(armMotor.getOutputCurrent()); // gets the velocity instead of current supply (cant find the function)
  }

  public SparkMaxAbsoluteEncoder getArmEncoder() {
    return armEncoder;
  }

  public PIDController getPID(){
    return armPID;
  }

  public void setRef(double ref){
    armPID.setSetpoint(ref);
  }

  public void setArmPower(double speed){
    armPID.setReference(speed, armEncoder.kPosition);
    armMotor.set(speed);
    armPID.
  }

  public double getAngle(double pos){
    double ang= pos*360;
    return ang;
  }

}
