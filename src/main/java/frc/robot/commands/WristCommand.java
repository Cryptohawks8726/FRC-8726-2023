// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

public class WristCommand extends CommandBase {
  private WristSubsystem subsystem;
  private XboxController xbox;

  public WristCommand(WristSubsystem m_subsystem, XboxController controller) {
    subsystem = m_subsystem;
    xbox = controller;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.kill();
  }

  @Override
  public void execute() {
    if(xbox.getLeftBumper()) {
      subsystem.raiseArm();
    } else if (xbox.getRightBumper()) {
      subsystem.lowerArm();
    } else {
      subsystem.kill();
    }
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.kill();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
