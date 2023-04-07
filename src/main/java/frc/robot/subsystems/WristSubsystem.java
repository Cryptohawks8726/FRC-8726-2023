// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntake;


public class WristSubsystem extends SubsystemBase {
    private SparkMaxAbsoluteEncoder encoder;
    private boolean isExtended;
    private CANSparkMax wristMotor;
    private SparkMaxPIDController posPid;
    private ProfiledPIDController pidFF;
    private ArmFeedforward arbFF;
    private DigitalInput limSwitch;
    private double retractPos;
    private ArmSubsystem arm;

    public WristSubsystem(ArmSubsystem arm) {
        this.arm = arm;
        limSwitch = new DigitalInput(0);
        isExtended = false;
        retractPos = ArmIntake.WRIST_RETRACT_POS; //0.9549
        pidFF = new ProfiledPIDController(6.0, 0, 0, new Constraints(-0.18, 0.22));// might need to change constraints bc units
        arbFF = new ArmFeedforward(0.0, 0.5, 0.35);

        wristMotor = new CANSparkMax(ArmIntake.WRIST_SPARKMAX, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setSmartCurrentLimit(30);

        encoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(1);
        encoder.setZeroOffset(ArmIntake.WRIST_ENCODER_OFFSET);
        
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // send can encoder pos data every 20ms
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // same but for vel
        wristMotor.setInverted(false);
        posPid = wristMotor.getPIDController();
        posPid.setPositionPIDWrappingEnabled(false);
        posPid.setP(ArmIntake.WRIST_kP);
        posPid.setI(ArmIntake.WRIST_kI);
        posPid.setD(ArmIntake.WRIST_kD);
        posPid.setFeedbackDevice(encoder);
        posPid.setOutputRange(-0.18, 0.22, 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Pos",encoder.getPosition());
        SmartDashboard.putNumber("wrist goal",pidFF.getSetpoint().position);
        SmartDashboard.putBoolean("Wrist Lim switch", limSwitch.get());
        if(!limSwitch.get() && !isExtended){ // if arm retracted
            setReference(encoder.getPosition());
            //pidFF.setGoal(encoder.getPosition())
            //System.out.println(Math.abs(-95.0-arm.getDegrees()));
            //System.out.println("Lim Switch Skip:"+(retractPos-encoder.getPosition()));
            if(Math.abs(-95.0-arm.getDegrees())<2.5){
            retractPos = encoder.getPosition();
            setReference(retractPos);
            //pidFF.setGoal(encoder.getPosition());
            }
        }
       // double pOut = pidFF.calculate(encoder.getPosition())
        //wristMotor.set(pOut+arbFF.calculate(pidFF.getSetpoint().position*2*Math.PI, pidFF.getSetpoint().velocity*2*Math.PI));
    }

    public double getRadians(){
        return (encoder.getPosition()-retractPos)*2*Math.PI;
    }
    public void setReference(double setPoint){
        posPid.setReference(setPoint, ControlType.kPosition);
        //pidFF.setGoal(setPoint);
    }

    public void retractWrist(){
        isExtended = false;
        posPid.setReference(retractPos-0.015, ControlType.kPosition);
        //posPid.setReference(-0.1, ControlType.kVelocity);
        SmartDashboard.putNumber("ref",retractPos);
    }



    public void fullyRetractWrist(){
        isExtended = false;
        posPid.setReference(retractPos-0.030, ControlType.kPosition);
        //posPid.setReference(-0.1, ControlType.kVelocity);
        SmartDashboard.putNumber("ref",retractPos);
    }

    public void extendWrist(){
        isExtended = true;
        posPid.setReference(retractPos+ArmIntake.EXTEND_DIFF, ControlType.kPosition);
        SmartDashboard.putNumber("ref",retractPos+ArmIntake.EXTEND_DIFF);
    }

    public void toggleExtend(){
        if(isExtended){
            retractWrist();
        }else{
            extendWrist();
        }
    } 
    
    public void shelfExtend(){
        isExtended = true;
        posPid.setReference(retractPos+ArmIntake.SHELF_DIFF, ControlType.kPosition);
        SmartDashboard.putNumber("ref",retractPos+ArmIntake.SHELF_DIFF);
    }

    public void stopWrist() {
        posPid.setReference(encoder.getPosition(), ControlType.kPosition);
    }

    public void setRetractPos(){
        retractPos = encoder.getPosition();
    }
}