// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;


public class RobotContainer {

  private final Compressor compressor;
  private final GroundIntakeSubsystem groundIntakeSubsystem;
  private final ArmIntakeSubsystem armIntakeSubsystem;
  private ArmSubsystem armSubsystem;
  private final SwerveDrive drivetrain;

  private final LED ledStrip = new LED(Constants.LED_PORT, Constants.LED_LENGTH);

  private CommandXboxController operatorController;
  private CommandJoystick driverJoystick;


  public RobotContainer() {

    
    drivetrain = new SwerveDrive();
    armIntakeSubsystem = new ArmIntakeSubsystem();
    compressor = new Compressor(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
    groundIntakeSubsystem = new GroundIntakeSubsystem();
    armSubsystem = new ArmSubsystem();
  
    driverJoystick = new CommandJoystick(Constants.DRIVER_CONTROLLER);
    operatorController = new CommandXboxController(Constants.OPERATOR_XBOX);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //drive cmds
    drivetrain.setDefaultCommand(new XboxTeleopDrive(drivetrain,driverJoystick).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Trigger driverRightBumper = driverController.rightBumper();
    // driverRightBumper.whileTrue(drivetrain.passiveBrake());
    // Trigger driverRightTrigger = driverController.rightTrigger();
    // driverRightTrigger.whileTrue(new RepeatCommand(new InstantCommand(()->drivetrain.normalZeroModules(),drivetrain)));
 
    // set arm pid for cone
    Trigger operatorBack = operatorController.back();
    operatorBack.onTrue(new InstantCommand(() -> {armSubsystem.coneHeld();}));
    // set arm pid for no cone or cube
    Trigger operatorStart = operatorController.start();
    operatorStart.onTrue(new InstantCommand(() -> {armSubsystem.coneNotHeld();}));
    
    // toggle wrist position
    Trigger driverThumb = driverJoystick.top();
    driverThumb.onTrue(new InstantCommand(()->{armIntakeSubsystem.toggleExtend();}));
    
    // raise arm intake at set vel
    Trigger driver3 = driverJoystick.button(3);
    driver3.whileTrue(new StartEndCommand(()->{armIntakeSubsystem.raiseIntake();}, ()->{armIntakeSubsystem.stopWrist();}, armIntakeSubsystem));

    // lower arm intake at set vel
    Trigger driver4 = driverJoystick.button(4);
    driver4.whileTrue(new StartEndCommand(()->{armIntakeSubsystem.lowerIntake();}, ()->{armIntakeSubsystem.stopWrist();}, armIntakeSubsystem));

    // raise arm at set vel
    Trigger driver5 = driverJoystick.button(5);
    driver5.whileTrue(new StartEndCommand(()->{armSubsystem.raiseArm();}, ()->{armSubsystem.stay(); }, armSubsystem));
  
    // lower arm at set vel
    Trigger driver6 = driverJoystick.button(6);
    driver6.whileTrue(new StartEndCommand(()->{armSubsystem.lowerArm();}, ()->{armSubsystem.stay();}, armSubsystem));

    // holding x lowers and opens ground intake, releasing it closes and stores ground intake
    Trigger operatorX = operatorController.x();
    operatorX.whileTrue(groundIntakeSubsystem.unstoreIntakeCmd()).onFalse(groundIntakeSubsystem.storeIntakeCmd());
    //x.whileTrue(new StartEndCommand(() -> {groundIntakeSubsystem.unstoreIntake();}, () -> {groundIntakeSubsystem.storeIntake();}, groundIntakeSubsystem));
    
    // holding y lowers and opens arm intake, releasing it closes and stores arm intake
    Trigger operatorY = operatorController.y();
    operatorY.whileTrue(armIntakeSubsystem.unstoreIntakeCmd()).onFalse(armIntakeSubsystem.storeIntakeCmd());
    //y.whileTrue(new StartEndCommand(() -> {armIntakeSubsystem.unstoreIntake();}, () -> {armIntakeSubsystem.storeIntake();}, armIntakeSubsystem));
  
    // turns off the LED strip
    Trigger driver7 = driverJoystick.button(7);
    driver7.onTrue(new InstantCommand(() -> {ledStrip.stop();}));

    // sets LED strip to yellow
    Trigger driver8 = driverJoystick.button(8);
    driver8.onTrue(new InstantCommand(() -> {ledStrip.setRGB(Constants.YELLOW_RGB);}));

    // sets LED strip to purple
    Trigger driver9 = driverJoystick.button(9);
    driver9.onTrue(new InstantCommand(() -> {ledStrip.setRGB(Constants.PURPLE_RGB);}));
  }
  //public Command getAutonomousCommand() {
  //  return armCommand;
 // }
}

