// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ArmIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  private final CommandJoystick driverJoystick = new CommandJoystick(Constants.DRIVER_CONTROLLER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERTOR_CONTROLLER);

  private final Compressor compressor = new Compressor(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
  private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
  private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();

  Trigger x, y;

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // holding x lowers and opens ground intake, releasing it closes and stores ground intake
    x = operatorController.x();
    x.whileTrue(new StartEndCommand(() -> {groundIntakeSubsystem.unstoreIntake();}, () -> {groundIntakeSubsystem.storeIntake();}, groundIntakeSubsystem));
    
    // holding y lowers and opens arm intake, releasing it closes and stores arm intake
    y = operatorController.y();
    y.whileTrue(new StartEndCommand(() -> {armIntakeSubsystem.unstoreIntake();}, () -> {armIntakeSubsystem.storeIntake();}, armIntakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
