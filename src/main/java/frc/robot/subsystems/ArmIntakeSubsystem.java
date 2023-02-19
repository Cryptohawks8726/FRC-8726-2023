// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmIntakeSubsystem extends SubsystemBase {
    private Solenoid lowerPiston;
    private Solenoid upperPiston;
    private SparkMaxAbsoluteEncoder encoder;
    private Timer timer;
    private boolean flag;
    private boolean isExtended;
    private CANSparkMax wristMotor;
    private SparkMaxPIDController posPid;

    public ArmIntakeSubsystem() {
        lowerPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.GROUND_INTAKE_LOWER_PISTON);
        upperPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.GROUND_INTAKE_UPPER_PISTON);
    
        timer = new Timer();
        timer.reset();
        timer.start();
        
        flag = true;
        isExtended = false;

        wristMotor = new CANSparkMax(Constants.WRIST_MOTOR_SPARKMAX, MotorType.kBrushed);
        wristMotor.setIdleMode(IdleMode.kCoast);

        encoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        
        posPid = wristMotor.getPIDController();
        posPid.setP(Constants.WRIST_kP);
        posPid.setI(Constants.WRIST_kI);
        posPid.setD(Constants.WRIST_kD);
        
        posPid.setFeedbackDevice(encoder);
    }

    @Override
    public void periodic() {
        if(flag) {
            //zeroEncoder();
            flag = false;
           }
        SmartDashboard.putNumber("Wrist Pos",encoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {}

    public void lowerPistonToggleCollapse(){
        if (lowerPiston.get()) {
            lowerPiston.set(true);
            upperPiston.set(true);
        } else {
            lowerPiston.set(false);
            upperPiston.set(false);
        }
    }

    public void openIntake() {
        lowerPiston.set(true);
        upperPiston.set(true);
    }

    public void closeIntake() {
        lowerPiston.set(false);
        upperPiston.set(false);
    }

    public double getEncoderPos() {
        return encoder.getPosition();
    }

    public void raiseIntake() {
        wristMotor.set(Constants.RAISE_WRIST_SPEED);
    }

    public void lowerIntake() {
        wristMotor.set(Constants.LOWER_WRIST_SPEED);
    }

    public void setReference(double setPoint){
        posPid.setReference(setPoint, ControlType.kPosition);
    }

    public void retract(){
        isExtended = false;
        posPid.setReference(Constants.WRIST_RETRACT_POS, ControlType.kPosition);
    }

    public void extend(){
        isExtended = true;
        posPid.setReference(Constants.WRIST_EXTEND_POS, ControlType.kPosition);
    }

    public void toggleExtend(){
        if(isExtended){
            retract();
        }else{
            extend();
        }
    } 

    public void stop() {
        wristMotor.set(0.0);
        posPid.setReference(encoder.getPosition(), ControlType.kPosition);
    }

    public void zeroEncoder(){ // remove once we have offsets. I don't see a use for generating new offsets every match
        encoder.setZeroOffset(encoder.getPosition() + encoder.getZeroOffset());
        SmartDashboard.putNumber("Wrist Encoder Offset", encoder.getZeroOffset());
        SmartDashboard.putNumber("Wrist Pos after Offset", encoder.getPosition()); // should be 0, double checking
    }

    public void zeroOffset(){
        encoder.setZeroOffset(Constants.WRIST_ENCODER_OFFSET);
    }

    public void unstoreIntake() {
        timer.start();
        lowerIntake();
        
        if (timer.get() > 0.5){
            openIntake();
        }

        timer.stop();
        timer.reset();
    }

    public void storeIntake() {
        closeIntake();
        timer.start();

        if (timer.get() > 0.5 ) {
            raiseIntake();
        }

        timer.stop();
        timer.reset();
    }
}