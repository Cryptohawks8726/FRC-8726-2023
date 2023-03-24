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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntake;


public class WristSubsystem extends SubsystemBase {
    private SparkMaxAbsoluteEncoder encoder;
    private boolean isExtended;
    private CANSparkMax wristMotor;
    private SparkMaxPIDController posPid;
    private DigitalInput limSwitch;
    private double retractPos;

    public WristSubsystem() {
       
        limSwitch = new DigitalInput(0);
        isExtended = false;
        retractPos = ArmIntake.WRIST_RETRACT_POS;

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
        posPid.setOutputRange(-0.1, 0.2, 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Pos",encoder.getPosition());
        SmartDashboard.putBoolean("Wrist Lim switch", limSwitch.get());
        if(!limSwitch.get() && !isExtended){
            System.out.println("Lim Switch Skip:"+(retractPos-encoder.getPosition()));
            retractPos = encoder.getPosition();
            setReference(retractPos);
        }
    }

    public void setReference(double setPoint){
        posPid.setReference(setPoint, ControlType.kPosition);
    }

    public void retractWrist(){
        isExtended = false;
        posPid.setReference(retractPos, ControlType.kPosition);
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
}