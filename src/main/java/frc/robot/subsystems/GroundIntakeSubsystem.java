package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

public class GroundIntakeSubsystem extends SubsystemBase{
    private Solenoid intakeSolenoid;
    private Solenoid GroundSolenoid;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private MotorControllerGroup LMcontroller;
    private MotorControllerGroup RMcontroller;

    private Timer timer;
    
    public GroundIntakeSubsystem() {

        leftMotor = new CANSparkMax(Constants.GROUND_INTAKE_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.GROUND_INTAKE_RIGHT_MOTOR, MotorType.kBrushless);

        LMcontroller = new MotorControllerGroup(leftMotor);
        RMcontroller = new MotorControllerGroup(rightMotor);

        timer = new Timer();
            
        intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.GROUND_INTAKE_LOWER_PISTON);
        GroundSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.GROUND_INTAKE_UPPER_PISTON);
    }

    public void wheelsIn() {
        LMcontroller.set(Constants.GROUND_INTAKE_WHEEL_SPEED);
        RMcontroller.set(-Constants.GROUND_INTAKE_WHEEL_SPEED);
    }
 
    public void wheelsOut() {
        LMcontroller.set(-Constants.GROUND_INTAKE_WHEEL_SPEED);
        RMcontroller.set(Constants.GROUND_INTAKE_WHEEL_SPEED);
    }

    public void wheelsOff() {
        LMcontroller.set(0);
        RMcontroller.set(0);
    }

    public void lowerIntake() {
        GroundSolenoid.set(true);
    }

    public void raiseIntake() {
        GroundSolenoid.set(false);
    }

    public void closeIntake() {
        intakeSolenoid.set(false);
    } 

    public void openIntake() {
        intakeSolenoid.set(true);
    }

    public void unstoreIntake() {
        timer.start();
        lowerIntake();
        
        if (timer.get() > 0.5){
            openIntake();
            wheelsIn();
        }

        timer.stop();
        timer.reset();
    }

    public void storeIntake() {
        closeIntake();
        timer.start();

        if (timer.get() > 0.5 ) {
            wheelsOff();
            raiseIntake();
        }

        timer.stop();
        timer.reset();
    }

    public void dropPiece() {
        timer.start();
        lowerIntake();

        if (timer.get() > 0.5) {
            openIntake();
        }

        timer.stop();
        timer.reset();
    }
}
