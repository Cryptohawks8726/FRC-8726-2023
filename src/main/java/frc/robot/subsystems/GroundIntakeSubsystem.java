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
import com.revrobotics.CANSparkMax.IdleMode;
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
    public boolean isOpen;
    
    public GroundIntakeSubsystem() {

        leftMotor = new CANSparkMax(GroundIntake.LEFT_SPARKMAX, MotorType.kBrushless);
        rightMotor = new CANSparkMax(GroundIntake.RIGHT_SPARKMAX, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(10);
        rightMotor.setSmartCurrentLimit(10);
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);
        isExtended = false;
        isOpen = false;
            
        intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, GroundIntake.CLAMP_PISTON);
        groundSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, GroundIntake.UPPER_PISTON,GroundIntake.LOWER_PISTON);
    }

    public void wheelsIn() {
       leftMotor.set(-0.1);
       rightMotor.set(-0.1); 
    }

    public void wheelsIntake(){
        leftMotor.set(-0.2);
        rightMotor.set(-0.2);
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

    public void toggleExtend(){
        if(isExtended){
            raiseIntake();
        }else{
            lowerIntake();
        }
    }
    public void closeIntake() {
        intakeSolenoid.set(false);
        isOpen = false;
    } 

    public void openIntake() {
        intakeSolenoid.set(true);
        isOpen = true;
    }

    public void toggleClamp(){
        if(isOpen){
            closeIntake();
            wheelsIn();
            //wheelsOff();
        } else{
            openIntake();
            wheelsIntake();
        }
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
