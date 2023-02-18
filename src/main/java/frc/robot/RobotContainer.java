// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  private CommandXboxController operatorController;
  private CommandJoystick driverJoystick;
  private ArmSubsystem armSubsystem;
  private WristSubsystem wristSubsystem;
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drivetrain;

  public RobotContainer() {

    
    drivetrain = new SwerveDrive();
  
    driverJoystick = new CommandJoystick(0);
    operatorController = new CommandXboxController(Constants.OPERATOR_XBOX);

    armSubsystem = new ArmSubsystem();
    wristSubsystem = new WristSubsystem();

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    Trigger operatorBack = operatorController.back();
    operatorBack.onTrue(new InstantCommand(() -> {armSubsystem.coneHeld();}));

    Trigger operatorStart = operatorController.start();
    operatorStart.onTrue(new InstantCommand(() -> {armSubsystem.coneNotHeld();}));
    
    // toggle wrist position
    Trigger driverThumb = driverJoystick.top();
    driverThumb.onTrue(new InstantCommand(()->{wristSubsystem.toggleExtend();}));
    
    // arm and wrist raise commands
    Trigger driver3 = driverJoystick.button(3);
    driver3.whileTrue(new StartEndCommand(()->{wristSubsystem.raiseArm();}, ()->{wristSubsystem.stop();}, wristSubsystem));

    Trigger driver4 = driverJoystick.button(4);
    driver4.whileTrue(new StartEndCommand(()->{wristSubsystem.lowerArm();}, ()->{wristSubsystem.stop();}, wristSubsystem));

    Trigger driver5 = driverJoystick.button(5);
    driver5.whileTrue(new StartEndCommand(()->{armSubsystem.raiseArm();}, ()->{armSubsystem.stay(); }, armSubsystem));
  
    drivetrain.setDefaultCommand(new XboxTeleopDrive(drivetrain,driverJoystick).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Trigger driverRightBumper = driverController.rightBumper();
    // driverRightBumper.whileTrue(drivetrain.passiveBrake());
    // Trigger driverRightTrigger = driverController.rightTrigger();
    // driverRightTrigger.whileTrue(new RepeatCommand(new InstantCommand(()->drivetrain.normalZeroModules(),drivetrain)));
 
    Trigger driver6 = driverJoystick.button(6);
    driver6.whileTrue(new StartEndCommand(()->{armSubsystem.lowerArm();}, ()->{armSubsystem.stay();}, armSubsystem));

  }

  //public Command getAutonomousCommand() {
  //  return armCommand;
 // }
}
