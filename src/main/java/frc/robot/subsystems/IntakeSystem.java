// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class IntakeSystem extends SubsystemBase {
  private boolean isCollapsed;

  private Solenoid extendSolenoid;
  private CommandXboxController m_controller;
  private Compressor pcmCompressor;
  private boolean enabled;
  private boolean pressureSwitch;
  
  public IntakeSystem(CommandXboxController xboxController) {

    // Generating and Storing Pressure
    m_controller = xboxController;
    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  
    pcmCompressor.enableDigital();
    pcmCompressor.disable();

    enabled = pcmCompressor.isEnabled();
    pressureSwitch = pcmCompressor.getPressureSwitchValue();

    // Single Solenoids
    // To set the value of the solenoid call set(true) to enable or set(false) to disable the solenoid output.
    extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);// add can id

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleCollapse() {
    if (isCollapsed) {
      extendSolenoid.set(true);
      isCollapsed = false;
    } else {
      extendSolenoid.set(false);
      isCollapsed = true;
    }
  }

  public void collapse() {
    extendSolenoid.set(false);
    isCollapsed = true;
  }

  public void extend() {
    extendSolenoid.set(true);
    isCollapsed = false;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void switchIntakeState() {
    isCollapsed = !isCollapsed;
  }

  public boolean isCollapsed() {
    return isCollapsed;
  }
}