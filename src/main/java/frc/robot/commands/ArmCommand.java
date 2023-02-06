// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmCommand extends CommandBase {
  private ArmSubsystem subsystem;
  private XboxController xbox;

  public ArmCommand(ArmSubsystem m_subsystem, XboxController controller) {
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

    boolean isConeHeld = false;
    SmartDashboard.putBoolean("Cone Held", isConeHeld);

    if(isConeHeld == true){
      subsystem.coneHeld();
    }
    else{
      subsystem.coneNotHeld();
    }

    if (xbox.getLeftBumperReleased() || xbox.getRightBumperReleased()) {
      subsystem.setRefPoint(subsystem.getEncoderPos());
      return;
    } 

    if (xbox.getLeftBumper()) {
      subsystem.lowerArm();
    } else if (xbox.getRightBumper()) {
      subsystem.raiseArm();
    } else {
      subsystem.stay();
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
