// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmCommand extends CommandBase {
  private ArmSubsystem m_subsystem;
  private XboxController xbox;
  

  public ArmCommand(ArmSubsystem subsystem, XboxController xboxC) {
    m_subsystem = subsystem;
    xbox = xboxC;
    addRequirements(subsystem);
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(xbox.getRawButton(XboxController.Button.kLeftBumper.value)) {
      m_subsystem.setArmPower(Constants.RAISE_ARM_SPEED);
    } else if (xbox.getRawButton(XboxController.Button.kRightBumper.value)) {
      m_subsystem.setArmPower(Constants.LOWER_ARM_SPEED);
    } else{
      m_subsystem.setArmPower(0.0);
      m_subsystem.setRef(m_subsystem.getAngle((Double)m_subsystem.getArmEncoder().getPosition()));
      m_subsystem.getPID().calculate(m_subsystem.getAngle((Double)m_subsystem.getArmEncoder().getPosition()));
      
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setArmPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
