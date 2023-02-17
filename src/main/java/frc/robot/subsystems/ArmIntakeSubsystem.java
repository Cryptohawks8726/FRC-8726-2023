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
  private boolean collapsed; 
  private Solenoid claw;
  private Compressor pcmCompressor;
  private boolean enabled;
  private boolean pressureSwitch;
  private CommandXboxController m_controller;

  /** Creates a new ExampleSubsystem. */
  public ArmIntakeSubsystem(CommandXboxController xboxController) {
    claw = new Solenoid(PneumaticsModuleType.REVPH, 0);
    collapsed = claw.get();
     // Generating and Storing Pressure
     m_controller = xboxController;
     pcmCompressor = new Compressor(0, PneumaticsModuleType.REVPH);
   
     pcmCompressor.enableDigital();
     //pcmCompressor.disable();
 
     enabled = pcmCompressor.isEnabled();
     pressureSwitch = pcmCompressor.getPressureSwitchValue();
  }

  public void toggleState() {
    collapsed = !collapsed;
  }

  public void toggleCollapse(){
    if (claw.get()) {
      claw.set(true);
    } else {
      claw.set(false);
    }
  }
  public void setTrue() {
    claw.set(true);
    collapsed = true;
  }
  public void setFalse() {
    claw.set(false);
    collapsed = false;
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
