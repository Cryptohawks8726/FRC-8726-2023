// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private XboxController driverController, operatorController;
  
  private ArmSubsystem armSubsytem;

  private ArmCommand armCommand;

  public RobotContainer() {
    driverController = new XboxController(Constants.DRIVER_XBOX);
    operatorController = new XboxController(Constants.OPERATOR_XBOX);

    armSubsytem = new ArmSubsystem();
    // handSubsytem = new HandSubsystem();

    armCommand = new ArmCommand(armSubsytem, driverController);
    // handCommand = new HandCommand(handSubsytem, driverController);

    configureButtonBindings();

    armSubsytem.setDefaultCommand(armCommand);
    // handSubsytem.setDefaultCommand(handCommand);
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return armCommand;
  }
}
