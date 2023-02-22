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

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmIntake;


public class ArmIntakeSubsystem extends SubsystemBase {
    private Solenoid piston;
    private SparkMaxAbsoluteEncoder encoder;
    private Timer timer;
    private boolean flag;
    private boolean isExtended;
    private CANSparkMax wristMotor;
    private SparkMaxPIDController posPid;

    public ArmIntakeSubsystem() {
        piston = new Solenoid(PneumaticsModuleType.REVPH, ArmIntake.PISTON);
    
        timer = new Timer();
        timer.reset();
        timer.start();
        
        flag = true;
        isExtended = false;

        wristMotor = new CANSparkMax(ArmIntake.WRIST_SPARKMAX, MotorType.kBrushed);
        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setSmartCurrentLimit(30);

        encoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(1);
        encoder.setZeroOffset(ArmIntake.WRIST_ENCODER_OFFSET);
        
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // send can encoder pos data every 20ms
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // same but for vel
        wristMotor.setInverted(true);
        posPid = wristMotor.getPIDController();
        posPid.setPositionPIDWrappingEnabled(false);
        posPid.setP(ArmIntake.WRIST_kP);
        posPid.setI(ArmIntake.WRIST_kI);
        posPid.setD(ArmIntake.WRIST_kD);
        //posPid.setOutputRange(-0.5, 0.5);
        posPid.setFeedbackDevice(encoder);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Pos",encoder.getPosition());
        
    }

    @Override
    public void simulationPeriodic() {}

    public void lowerPistonToggleCollapse(){
        if (piston.get()) {
            piston.set(false);
        } else {
            piston.set(true);
        }
    }

    public void openIntake() {
        piston.set(true);
    }

    public void closeIntake() {
        piston.set(false);
    }

    public void raiseIntake() {
        posPid.setReference(ArmIntake.RAISE_WRIST_SPEED, ControlType.kVelocity);
        //wristMotor.set(Constants.RAISE_WRIST_SPEED);
    }

    public void lowerIntake() {
        posPid.setReference(ArmIntake.LOWER_WRIST_SPEED, ControlType.kVelocity);
        //wristMotor.set(Constants.LOWER_WRIST_SPEED);
    }

    public void setReference(double setPoint){
        posPid.setReference(setPoint, ControlType.kPosition);
    }

    public void retractWrist(){
        isExtended = false;
        posPid.setReference(ArmIntake.WRIST_RETRACT_POS, ControlType.kPosition);
        SmartDashboard.putNumber("ref",ArmIntake.WRIST_RETRACT_POS);
        System.out.println("printy");
    }

    public void extendWrist(){
        isExtended = true;
        posPid.setReference(ArmIntake.WRIST_EXTEND_POS, ControlType.kPosition);
        SmartDashboard.putNumber("ref",ArmIntake.WRIST_EXTEND_POS);
    }

    public void toggleExtend(){
        if(isExtended){
            retractWrist();
        }else{
            extendWrist();
        }
    } 

    public void stopWrist() {
        //wristMotor.set(0.0);
        posPid.setReference(encoder.getPosition(), ControlType.kPosition);
    }

    public void zeroEncoder(){ // remove once we have offsets. I don't see a use for generating new offsets every match
        encoder.setZeroOffset(encoder.getPosition() + encoder.getZeroOffset());
        SmartDashboard.putNumber("Wrist Encoder Offset", encoder.getZeroOffset());
        SmartDashboard.putNumber("Wrist Pos after Offset", encoder.getPosition()); // should be 0, double checking
    }

    public void zeroOffset(){
        encoder.setZeroOffset(ArmIntake.WRIST_ENCODER_OFFSET);
    }

    public void setSpeed(double speed){
        wristMotor.set(speed);
    }
    /*public void unstoreIntake() {
        timer.start();
        lowerIntake();
        
        if (timer.get() > 0.5){
            openIntake();
        }

        timer.stop();
        timer.reset();
    }*/

    public SequentialCommandGroup unstoreIntakeCmd(){
        return new InstantCommand(()->{this.extendWrist();}, this) //change to this.extendWrist later
        .andThen(new WaitCommand(0.5))
        .andThen(()->{this.stopWrist();}) // removed when change to extend
        .andThen(()->{this.openIntake();});
    }

    /*public void storeIntake() {
        closeIntake();
        timer.start();

        if (timer.get() > 0.5 ) {
            raiseIntake();
        }

        timer.stop();
        timer.reset();
    }*/

    public SequentialCommandGroup storeIntakeCmd(){
        return new InstantCommand(()->{this.closeIntake();},this) 
        .andThen(new WaitCommand(0.5)) 
        .andThen(()->{this.retractWrist();}) // change to this.retractWrist later
        .andThen(new WaitCommand(0.5)) //remove after change
        .andThen(()->{this.stopWrist();}) //remove after change
        ;
    }
}