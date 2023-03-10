package frc.robot.subsystems;


import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmIntake2Subsystem extends SubsystemBase {
    private CANSparkMax motor;
    //private boolean isHeld;
    private String motorStatus = "Stopped";


    public ArmIntake2Subsystem() {
        //isHeld = false;

        motor = new CANSparkMax(Constants.ArmIntake.ARM_INTAKE_SPARKMAX, MotorType.kBrushless);

        // Set Current Limits
        motor.setSmartCurrentLimit(20);

        motor.setIdleMode(IdleMode.kBrake);

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("motorStatus", motorStatus);
    }

    public InstantCommand intake() {
        return new InstantCommand(()->{runIntake();}, this);
    }

    public InstantCommand eject() {
        return new InstantCommand(()->{ejectIntake();}, this);
    }

    public InstantCommand stop() {
        return new InstantCommand(()->{stopIntake();}, this);
    }

    public void runIntake(){
        motor.set(2);
        motorStatus = "Intaking...";
    }

    public void ejectIntake(){
        motor.set(-2);
        motorStatus = "Ejecting";
    }

    public void stopIntake(){
        motor.set(0);
        motorStatus = "Stopped..";
    }
}
