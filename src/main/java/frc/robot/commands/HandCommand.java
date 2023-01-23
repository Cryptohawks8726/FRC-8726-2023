// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class HandCommand extends CommandBase {
  private HandSubsystem m_subsystem;
  private XboxController xbox;

  public HandCommand(HandSubsystem subsystem, XboxController xbox) {
    m_subsystem = subsystem;
    this.xbox = xbox;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(xbox.getRawButton(XboxController.Button.kLeftBumper.value)) {
      m_subsystem.setHandPower(Constants.RAISE_ARM_SPEED);
    } else if (xbox.getRawButton(XboxController.Button.kRightBumper.value)) {
      m_subsystem.setHandPower(Constants.LOWER_ARM_SPEED);
    } else {
      m_subsystem.setHandPower(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setHandPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
