// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private XboxController driverController, operatorController;
  
  private ArmSubsystem armSubsystem;
  private WristSubsystem wristSubsystem;

  private ArmCommand armCommand;
  private WristCommand wristCommand;

  public RobotContainer() {
    driverController = new XboxController(Constants.DRIVER_XBOX);
    operatorController = new XboxController(Constants.OPERATOR_XBOX);

    armSubsystem = new ArmSubsystem();
    armCommand = new ArmCommand(armSubsystem, driverController);

    wristSubsystem = new WristSubsystem();
    wristCommand = new WristCommand(wristSubsystem, operatorController);

    configureButtonBindings();

    armSubsystem.setDefaultCommand(armCommand);
    wristSubsystem.setDefaultCommand(wristCommand);
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return armCommand;
  }
}
