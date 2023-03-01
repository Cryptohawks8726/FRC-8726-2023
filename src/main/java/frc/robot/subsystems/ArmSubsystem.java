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

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends SubsystemBase {
  private SparkMaxAbsoluteEncoder encoder;
  private Timer timer;
  private boolean isConeHeld;
  private SparkMaxPIDController posPID;
  private ArmFeedforward arbFF;
  private CANSparkMax armMotor;
  private Solenoid brake;

  public ArmSubsystem() {
    isConeHeld = false;
    timer = new Timer();

    armMotor = new CANSparkMax(Arm.ARM_SPARKMAX, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.enableVoltageCompensation(12.0);

    encoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    encoder.setZeroOffset(Arm.ENCODER_OFFSET_CONFIG);
    encoder.setPositionConversionFactor(Arm.ENCODER_POS_FACTOR);

    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // send can encoder pos data every 20ms
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // same but for vel

    arbFF = new ArmFeedforward(Arm.ARM_kS,Arm.ARM_kG,Arm.ARM_kV);
    posPID = armMotor.getPIDController();
    posPID.setP(Arm.ARM_kVP);
    posPID.setI(Arm.ARM_kVI);
    posPID.setD(Arm.ARM_kVD);
    posPID.setFeedbackDevice(encoder);
    posPID.setOutputRange(-0.05, 0.2, 0);
    
    brake = new Solenoid(PneumaticsModuleType.REVPH, Arm.BRAKE_PISTON);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Pos",getRawEncoderPos());
    SmartDashboard.putNumber("Arm Deg",getDegrees());
    SmartDashboard.putNumber("Corrected Deg to Ref",getDegrees()+Arm.ENCODER_OFFSET_SUBTRACT);
    SmartDashboard.putNumber("Applied Output",armMotor.getAppliedOutput());
  }

  public SequentialCommandGroup setRawPosRefPoint(double pos){
    return new InstantCommand(()->{releaseBrake();},this)
    .andThen(()->{posPID.setReference(getRawEncoderPos()+10, ControlType.kVoltage);})
    .andThen(()->setFF(getRawEncoderPos() + 10 - Arm.ENCODER_OFFSET_SUBTRACT))
    .andThen(new WaitCommand(0.5))
    .andThen(()->{posPID.setReference(pos, ControlType.kVoltage);})
    .andThen(()->setFF(pos-Arm.ENCODER_OFFSET_SUBTRACT));
  }

  public SequentialCommandGroup setDegPosRefPoint(double deg){
    return new InstantCommand(()->{releaseBrake();},this)
    .andThen(()->{setDegRefPoint(getDegrees());})
    //.andThen(new WaitCommand(0.5))
   // .andThen(()->{posPID.setReference(deg+Arm.ENCODER_OFFSET_SUBTRACT, ControlType.kVoltage);})
    .andThen(()->{setFF(deg);});
  }


 public void setDegRefPoint(double degrees){
    releaseBrake();
    if(degrees<-88){
      degrees = -88;
    }
    if(degrees>10){
      degrees = 10;
    }
    posPID.setReference(degrees+Arm.ENCODER_OFFSET_SUBTRACT, ControlType.kVoltage);
    setFF(degrees);
  }

  public void setFF(double degrees){
    posPID.setFF(arbFF.calculate(Math.toRadians(degrees), Arm.SPEED_RAD));
  }


  /*public void setVelRefPoint(double vel){
    posPID.setReference(vel, ControlType.kVelocity);
  }*/
  
  public double getRawEncoderPos() {
    return encoder.getPosition();
  }

  public double getDegrees(){
    return encoder.getPosition() - Arm.ENCODER_OFFSET_SUBTRACT;
  }

  public void raiseArm() {
    releaseBrake();
    setDegRefPoint(getDegrees()+2);
    //posPid.setReference()
    // setVelRefPoint(Arm.RAISE_ARM_SPEED);
    //armMotor.set(Arm.RAISE_ARM_SPEED);
  }

  public void lowerArm() {
    releaseBrake();

    setDegRefPoint(getDegrees()-2);
    
    //armMotor.set(Arm.LOWER_ARM_SPEED);
  }

  public void coneNotHeld() {
    isConeHeld = true;
    posPID.setP(Arm.ARM_kP);
    posPID.setI(Arm.ARM_kI);
    posPID.setD(Arm.ARM_kD);
  }

  public void coneHeld() {
    isConeHeld = false;
    posPID.setP(Arm.CONEHELD_kP);
    posPID.setI(Arm.CONEHELD_kI);
    posPID.setD(Arm.CONEHELD_kD);
  }

  public void stay() {
    //setRawPosRefPoint(getRawEncoderPos());
    setDegRefPoint(getDegrees());
    setFF(getDegrees());
    //armMotor.set(-posPID.calculate(encoderVal)); // why is this negative? we can use the 
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

  public SequentialCommandGroup setBrake(){
    return new InstantCommand(()->{this.stay();},this)
    .andThen(new WaitCommand(0.15))
    .andThen(()->{this.enableBrake();})
    .andThen(()->{this.posPID.setReference(getRawEncoderPos(), ControlType.kPosition);});
    //brake.set(false);
    //stay();
  }

  public void enableBrake(){
    brake.set(true);
  }
  public void releaseBrake(){
    brake.set(false);
  }
}
