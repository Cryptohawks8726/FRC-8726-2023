package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.GroundIntake;

public class GroundIntakeSubsystem extends SubsystemBase{
    private Solenoid intakeSolenoid;
    private DoubleSolenoid groundSolenoid;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    public boolean isExtended;
    
    public GroundIntakeSubsystem() {

        leftMotor = new CANSparkMax(GroundIntake.LEFT_SPARKMAX, MotorType.kBrushless);
        rightMotor = new CANSparkMax(GroundIntake.RIGHT_SPARKMAX, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(20);
        rightMotor.setSmartCurrentLimit(20);
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
        isExtended = false;
            
        intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, GroundIntake.CLAMP_PISTON);
        groundSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, GroundIntake.UPPER_PISTON,GroundIntake.LOWER_PISTON);
    }

    public void wheelsIn() {
        leftMotor.set(-GroundIntake.WHEEL_SPEED);
        rightMotor.set(-GroundIntake.WHEEL_SPEED); 
    }
 
    public void wheelsOut() {
        leftMotor.set(GroundIntake.WHEEL_SPEED);
        rightMotor.set(GroundIntake.WHEEL_SPEED);
    }

    public void wheelsOff() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void lowerIntake() {
        groundSolenoid.set(Value.kForward);
        isExtended = true;
    }

    public void raiseIntake() {
        groundSolenoid.set(Value.kReverse);
        isExtended = false;
    }

    public void closeIntake() {
        intakeSolenoid.set(false);
    } 

    public void openIntake() {
        intakeSolenoid.set(true);
    }

    public SequentialCommandGroup unstoreIntakeCmd(){
        return new InstantCommand(()->{this.lowerIntake();}, this)
        .andThen(new WaitCommand(0.25))
        .andThen(()->{this.openIntake();})
        .andThen(()->{this.wheelsIn();});
    }

    public SequentialCommandGroup storeIntakeCmd(){
        return new InstantCommand(()->{this.closeIntake();}, this)
        .andThen(new WaitCommand(0.25))
        .andThen(()->{this.wheelsOff();})
        .andThen(()->{this.raiseIntake();});
    }

    public SequentialCommandGroup dropPiece(){
        return new InstantCommand(()->{this.lowerIntake();},this)
        .andThen(new WaitCommand(1.75))
        .andThen(()->{this.openIntake();})
        .andThen(()->{this.wheelsOut();});
    }   
}
