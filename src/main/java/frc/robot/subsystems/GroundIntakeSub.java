package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;

public class GroundIntakeSub extends SubsystemBase{

    private Compressor pcmCompressor;
    private boolean enabled;
    private boolean pressureSwitch;
    private Solenoid intakeSolenoid;
    private Solenoid GroundSolenoid;
    private CommandXboxController m_controller;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private MotorControllerGroup LMcontroller;
    private MotorControllerGroup RMcontroller;

    private Timer timer;
    
    public GroundIntakeSub(Compressor pCompressor) {

        leftMotor = new CANSparkMax(1, MotorType.kBrushless);
        rightMotor = new CANSparkMax(0, MotorType.kBrushless);
        LMcontroller = new MotorControllerGroup(leftMotor);
        RMcontroller = new MotorControllerGroup(rightMotor);
        timer = new Timer();
        

        
        pcmCompressor = pCompressor;
    
        //pcmCompressor.enableDigital();
        //pcmCompressor.disable();

        enabled = pcmCompressor.isEnabled();
        pressureSwitch = pcmCompressor.getPressureSwitchValue();

        // Single Solenoids
        // To set the value of the solenoid call set(true) to enable or set(false) to disable the solenoid output.
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);// add can id
        GroundSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    }

    public void wheelsIn() {
        LMcontroller.set(0.3);
        RMcontroller.set(-0.3);
    }
 
    public void wheelsOut() {
        LMcontroller.set(-0.3);
        RMcontroller.set(0.3);

    }

    public void wheelsOff() {
        LMcontroller.set(0);
        RMcontroller.set(0);
    }

    public void intakeDown() {
        GroundSolenoid.set(true);
    }

    public void intakeUp() {
        GroundSolenoid.set(false);
    }


    public void CloseIntake() {
        intakeSolenoid.set(false);

    } 

    public void OpenIntake() {
        intakeSolenoid.set(true);
        
    }

    public void PickPiecePos() {
        timer.start();
        intakeDown();
        
        if (timer.get() > 0.5){
            OpenIntake();
            wheelsIn();
        }
        timer.stop();
        timer.reset();
    }

    public void CapturePiecePos() {
        CloseIntake();
        timer.start();
        if (timer.get() > 0.5 ) {
            wheelsOff();
            intakeUp();
        }
        timer.stop();
        timer.reset();

    }

    public void DropPiece() {
        timer.start();
        intakeDown();
        if (timer.get() > 0.5) {
            OpenIntake();
        }
        timer.stop();
        timer.reset();

    }

    public void BackIn() {
        timer.start();
        CloseIntake();
        if (timer.get() > 0.5) {
            intakeUp();
        }
        timer.stop();
        timer.reset();
    }

    public StartEndCommand capGamePiece() {
        return new StartEndCommand(()->{PickPiecePos();},()->{CapturePiecePos();}, this);
    }

    public StartEndCommand dropPiece() {
        return new StartEndCommand(()->{DropPiece();},()->{BackIn();}, this);   
    }

}
