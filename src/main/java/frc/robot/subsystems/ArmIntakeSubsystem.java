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
import frc.robot.Constants.ArmIntake;;


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
        lowerPiston = new Solenoid(PneumaticsModuleType.REVPH, ArmIntake.LOWER_PISTON);
        upperPiston = new Solenoid(PneumaticsModuleType.REVPH, ArmIntake.UPPER_PISTON);
    
        timer = new Timer();
        timer.reset();
        timer.start();
        
        flag = true;
        isExtended = false;

        wristMotor = new CANSparkMax(ArmIntake.WRIST_SPARKMAX, MotorType.kBrushed);
        wristMotor.setIdleMode(IdleMode.kCoast);

        encoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // send can encoder pos data every 20ms
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // same but for vel

        posPid = wristMotor.getPIDController();
        posPid.setP(ArmIntake.WRIST_kP);
        posPid.setI(ArmIntake.WRIST_kI);
        posPid.setD(ArmIntake.WRIST_kD);
        
        posPid.setFeedbackDevice(encoder);
    }

    @Override
    public void periodic() {
        if(flag) {
            //zeroEncoder();
            flag = false;
           }
        SmartDashboard.putNumber("Wrist Pos",encoder.getPosition());
        SmartDashboard.putNumber("Wrist Deg", encoder.getPosition()*360);
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
    }

    public void extendWrist(){
        isExtended = true;
        posPid.setReference(ArmIntake.WRIST_EXTEND_POS, ControlType.kPosition);
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
        return new InstantCommand(()->{this.lowerIntake();}, this) //change to this.extendWrist later
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
        .andThen(()->{this.raiseIntake();}) // change to this.retractWrist later
        .andThen(new WaitCommand(0.5)) //remove after change
        .andThen(()->{this.stopWrist();}) //remove after change
        ;
    }
}