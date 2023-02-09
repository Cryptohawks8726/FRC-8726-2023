// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmIntakeSubsystem extends SubsystemBase {
  private boolean collapsed = false; 
  private Solenoid claw;
  private Compressor pcmCompressor;
  private boolean enabled;
  private boolean pressureSwitch;
  private Solenoid solenoid1;
  private Solenoid exampleSingle;
  private CommandXboxController m_controller;

  /** Creates a new ExampleSubsystem. */
  public ArmIntakeSubsystem(CommandXboxController xboxController) {
    claw = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
     // Generating and Storing Pressure
     m_controller = xboxController;
     pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
   
     pcmCompressor.enableDigital();
     //pcmCompressor.disable();
 
     enabled = pcmCompressor.isEnabled();
     pressureSwitch = pcmCompressor.getPressureSwitchValue();
  }

  public void toggleState() {
    collapsed = !collapsed;
  }

  public void toggleCollapse(){
    System.out.println("Before:");
    //System.out.println(collapsed);
    System.out.println(claw.get());
    if (claw.get()) {
      claw.set(false);
    } else {
      claw.set(true);
    }
    System.out.println("After:");
    //System.out.println(collapsed);
    System.out.println(claw.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
