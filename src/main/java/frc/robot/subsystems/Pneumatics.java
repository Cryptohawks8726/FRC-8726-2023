package frc.robot.subsystems;

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

public class Pneumatics extends SubsystemBase { 
  /** Creates a new ExampleSubsystem. */
 //public Pneumatics() {
  //}

  private Compressor pcmCompressor;
  private boolean enabled;
  private boolean pressureSwitch;
  private Solenoid solenoid1;
  private Solenoid exampleSingle;
  private CommandXboxController m_controller;

  //@Override
  public Pneumatics(CommandXboxController xboxController) {
    // Generating and Storing Pressure
    m_controller = xboxController;
    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  
    pcmCompressor.enableDigital();
    pcmCompressor.disable();

    enabled = pcmCompressor.isEnabled();
    pressureSwitch = pcmCompressor.getPressureSwitchValue();

    // Single Solenoids
    // To set the value of the solenoid call set(true) to enable or set(false) to disable the solenoid output.
    solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);// add can id

    // Toggling Solenoids
    // Solenoids can be switched from one output to the other (known as toggling) by using the .toggle() method.
    // exampleSingle = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    // roboRIO
    // A pressure transducer can be connected to the Analog Input ports on the roboRIO, 
    // and can be read by the AnalogInput or AnalogPotentiometer classes in WPILib.

    // product-specific voltage->pressure conversion, see product manual
    // in this case, 250(V/5)-25
    // the scale parameter in the AnalogPotentiometer constructor is scaled from 1 instead of 5,
    // so if r is the raw AnalogPotentiometer output, the pressure is 250r-25
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // exampleSolenoidPCM.set(false);  

    if (m_controller.y().getAsBoolean() && !solenoid1.get()) {
      solenoid1.set(true);
    } else if (!m_controller.y().getAsBoolean() && solenoid1.get()) {
      solenoid1.set(false);
    }
    if (m_controller.x().getAsBoolean() && !pcmCompressor.isEnabled()) {
      pcmCompressor.enableDigital();
    }
    if (m_controller.b().getAsBoolean() && pcmCompressor.isEnabled()) {
      pcmCompressor.disable();
    }
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

