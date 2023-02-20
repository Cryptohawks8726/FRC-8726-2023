// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ArmIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final CommandJoystick driverJoystick = new CommandJoystick(Constants.DRIVER_CONTROLLER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERTOR_CONTROLLER);

  private final Compressor compressor = new Compressor(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
  private final GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
  private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();

  private final LED ledStrip = new LED(Constants.LED_PORT, Constants.LED_LENGTH);

  Trigger x, y, driver3, driver5, driver6;

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // turns off the LED strip
    driver3 = driverJoystick.button(3);
    driver3.onTrue(new InstantCommand(() -> {ledStrip.stop();}));

    // sets LED strip to yellow
    driver5 = driverJoystick.button(5);
    driver3.onTrue(new InstantCommand(() -> {ledStrip.setRGB(Constants.YELLOW_RGB);}));

    // sets LED strip to purple
    driver6 = driverJoystick.button(6);
    driver3.onTrue(new InstantCommand(() -> {ledStrip.setRGB(Constants.PURPLE_RGB);}));

    // holding x lowers and opens ground intake, releasing it closes and stores ground intake
    x = operatorController.x();
    x.whileTrue(groundIntakeSubsystem.unstoreIntakeCmd()).onFalse(groundIntakeSubsystem.storeIntakeCmd());
    //x.whileTrue(new StartEndCommand(() -> {groundIntakeSubsystem.unstoreIntake();}, () -> {groundIntakeSubsystem.storeIntake();}, groundIntakeSubsystem));
    
    // holding y lowers and opens arm intake, releasing it closes and stores arm intake
    y = operatorController.y();
    y.whileTrue(armIntakeSubsystem.unstoreIntakeCmd()).onFalse(armIntakeSubsystem.storeIntakeCmd());
    //y.whileTrue(new StartEndCommand(() -> {armIntakeSubsystem.unstoreIntake();}, () -> {armIntakeSubsystem.storeIntake();}, armIntakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
