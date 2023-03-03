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

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends SubsystemBase {
  private SparkMaxAbsoluteEncoder encoder;
  private Timer timer;
  private boolean isConeHeld;
  private SparkMaxPIDController posPID;
  private CANSparkMax armMotor;
  private Solenoid brake;

  public ArmSubsystem() {
    isConeHeld = false;
    timer = new Timer();
    //encoder = new DutyCycleEncoder(Constants.ARM_ENCODER_CHANNEL); // if encoder is wired to rio

    armMotor = new CANSparkMax(Arm.ARM_SPARKMAX, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kCoast);

    encoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    encoder.setZeroOffset(Arm.ENCODER_OFFSET_CONFIG);
    encoder.setPositionConversionFactor(Arm.ENCODER_POS_FACTOR);

    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // send can encoder pos data every 20ms
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // same but for vel

    //posPID = new PIDController(Constants.ARM_kP, Constants.ARM_kI, Constants.ARM_kD);
    posPID = armMotor.getPIDController();
    posPID.setP(Arm.ARM_kP);
    posPID.setI(Arm.ARM_kI);
    posPID.setD(Arm.ARM_kD);
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
    .andThen(()->{posPID.setReference(getRawEncoderPos()+10, ControlType.kPosition);})
    .andThen(new WaitCommand(0.5))
    .andThen(()->{posPID.setReference(pos, ControlType.kPosition);});
  }

  public SequentialCommandGroup setDegPosRefPoint(double deg){
    return new InstantCommand(()->{releaseBrake();},this)
    .andThen(()->{setDegRefPoint(getDegrees() + 10);})
    .andThen(new WaitCommand(0.5))
    .andThen(()->{posPID.setReference(deg+Arm.ENCODER_OFFSET_SUBTRACT, ControlType.kPosition);});
  }


 public void setDegRefPoint(double degrees){
    releaseBrake();
    if(degrees<-88){
      degrees = -88;
    }
    if(degrees>10){
      degrees = 10;
    }
    posPID.setReference(degrees+Arm.ENCODER_OFFSET_SUBTRACT, ControlType.kPosition);
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

  public CANSparkMax getSparkErrorArmMotor(){
    return armMotor;
  }
  public void enableBrake(){
    brake.set(true);
  }
  public void releaseBrake(){
    brake.set(false);
  }
}
