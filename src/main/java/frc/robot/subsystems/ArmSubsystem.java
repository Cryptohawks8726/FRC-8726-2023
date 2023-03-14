// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends SubsystemBase {
  private SparkMaxAbsoluteEncoder encoder;
  private ProfiledPIDController controller;
  private Constraints armConstraints;
  private ArmFeedforward arbFF;
  private double tempFF,lastSetRad,setVolt;
  private CANSparkMax armMotor;
  private Solenoid brake;

  public ArmSubsystem() {


    armMotor = new CANSparkMax(Arm.ARM_SPARKMAX, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kCoast);
    armMotor.enableVoltageCompensation(12.0);

    encoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    encoder.setZeroOffset(Arm.ENCODER_OFFSET_CONFIG);
    encoder.setPositionConversionFactor(Arm.ENCODER_POS_FACTOR);

    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // send can encoder pos data every 20ms
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // same but for vel
    tempFF = 0.0;
    setVolt = 0.43;
    arbFF = new ArmFeedforward(Arm.ARM_kS,Arm.ARM_kG,Arm.ARM_kV);
    armConstraints = new Constraints(0.15, 0.05); // dg/s
    controller = new ProfiledPIDController(Arm.ARM_kVP, Arm.ARM_kVI, Arm.ARM_kVD, armConstraints,0.2);
    controller.setGoal(getRadians());
    /*posPID = armMotor.getPIDController();
    posPID.setP(Arm.ARM_kVP);
    posPID.setI(Arm.ARM_kVI);
    posPID.setD(Arm.ARM_kVD);
    posPID.setFeedbackDevice(encoder);
    posPID.setOutputRange(-0.05, 0.2, 0);*/
    
    brake = new Solenoid(PneumaticsModuleType.REVPH, Arm.BRAKE_PISTON);
  }


  @Override
  public void periodic() {
  
    double pidOutput = controller.calculate(getRadians());
    double ff = arbFF.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity);
    // change to radians + change pid to radians
    armMotor.setVoltage(pidOutput+ff);

    
    SmartDashboard.putNumber("PID Effort",pidOutput);
    SmartDashboard.putNumber("FF",ff);
    SmartDashboard.putNumber("Arm Deg",getDegrees());
    SmartDashboard.putNumber("Applied Voltage",pidOutput+ff);
    SmartDashboard.putNumber("PID Vel",controller.getSetpoint().velocity);
  }

  public void setGoal(double deg){
    SmartDashboard.putNumber("last set deg",deg);
    controller.setGoal(Math.toRadians(deg));
  }

  public double getFF(double degrees,double vel){
    lastSetRad = Math.toRadians(degrees);
    tempFF = arbFF.calculate(lastSetRad, vel);
  
    SmartDashboard.putNumber("tempFF",tempFF);
    return tempFF;
  }

  public double getRawEncoderPos() {
    return encoder.getPosition();
  }

  public double getDegrees(){
    return encoder.getPosition() - Arm.ENCODER_OFFSET_SUBTRACT;
  }
  
  public double getRadians(){
    return Math.toRadians(getDegrees());
  }

  /* 
  calculate arm angle in deg to acheive height in inches
  0 deg is parallel to the ground
  - down
  */
  public double calcAngle(double height){ 
    double y = height-Arm.SHAFT_HEIGHT_INCHES;
    double x = Math.sqrt(Math.pow(Arm.ARM_LENGTH_INCHES,2)-Math.pow(y, 2));
    return Math.atan2(y, x)*180.0/Math.PI;
  }

  public void enableBrake(){
    brake.set(true);
  }

  public void releaseBrake(){
    brake.set(false);
  }
}
