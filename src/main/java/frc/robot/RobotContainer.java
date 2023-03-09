// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmIntake2Subsystem;
import frc.robot.Constants.Arm;
import frc.robot.commands.Balance;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class RobotContainer {

  private final PneumaticHub pneumaticHub;
  private  GroundIntakeSubsystem groundIntakeSubsystem;
  private ArmIntake2Subsystem armIntakeSubsystem;
  private ArmSubsystem armSubsystem;
 private final SwerveDrive drivetrain;

  private final LED ledStrip = new LED(Constants.LED_PORT, Constants.LED_LENGTH);

  private CommandXboxController operatorController;
  private CommandJoystick driverJoystick;


  public RobotContainer() {

    drivetrain = new SwerveDrive();
    armIntakeSubsystem = new ArmIntake2Subsystem();
    pneumaticHub = new PneumaticHub(Constants.COMPRESSOR_ID);
    //pneumaticHub.disableCompressor();
    pneumaticHub.enableCompressorDigital();
    //groundIntakeSubsystem = new GroundIntakeSubsystem();
    armSubsystem = new ArmSubsystem();
  
    driverJoystick = new CommandJoystick(Constants.DRIVER_CONTROLLER);
    operatorController = new CommandXboxController(Constants.OPERATOR_XBOX);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //armIntakeSubsystem.setDefaultCommand(new InstantCommand(()->{armIntakeSubsystem.retractWrist();},armIntakeSubsystem));
    //groundIntakeSubsystem.setDefaultCommand(groundIntakeSubsystem.storeIntakeCmd());
    //drive cmds
    drivetrain.setDefaultCommand(new XboxTeleopDrive(drivetrain,driverJoystick).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    //Trigger driver11 = driverJoystick.button(11);
    //driver11.whileTrue(new RepeatCommand(new InstantCommand(()->{drivetrain.normalZeroModules();},drivetrain))/*new InstantCommand(()->{drivetrain.drive(new ChassisSpeeds(1, 0, 0), false);}*/)
    //.onFalse(new InstantCommand(()->{drivetrain.drive(new ChassisSpeeds(0, 0, 0), false);}));

    
    // Trigger driverRightBumper = driverController.rightBumper();
    // driverRightBumper.whileTrue(drivetrain.passiveBrake());
    // Trigger driverRightTrigger = driverController.rightTrigger();
    // driverRightTrigger.whileTrue(new RepeatCommand(new InstantCommand(()->drivetrain.normalZeroModules(),drivetrain)));
    setArmBindings();
    //setGroundIntakeBindings();
    setLedBindings();

    Trigger operatorY = operatorController.y();
    //operatorY.whileTrue(new InstantCommand(()->{armIntakeSubsystem.extendWrist();},armIntakeSubsystem).withName("Extend Arm Intake"))
     // .onFalse(new InstantCommand(()->{armIntakeSubsystem.retractWrist();},armIntakeSubsystem).withName("Retract Arm Intake"));

    
    //operatorY.whileTrue(armIntakeSubsystem.unstoreIntakeCmd().withName("Unstore Arm Intake"))
    //.onFalse(armIntakeSubsystem.storeIntakeCmd().withName("Store Arm Intake"));
    

    

  }

  private void setArmBindings(){
    // set arm pid for cone
    Trigger operatorBack = operatorController.back();
    operatorBack.onTrue(new InstantCommand(() -> {armSubsystem.coneHeld();}));

    // set arm pid for no cone or cube
    Trigger operatorStart = operatorController.start();
    operatorStart.onTrue(new InstantCommand(() -> {armSubsystem.coneNotHeld();}));
    
    // toggle wrist position
    //Trigger driverThumb = driverJoystick.top();
    //driverThumb.onTrue(new InstantCommand(()->{armIntakeSubsystem.toggleExtend();},armIntakeSubsystem));

    // holding y lowers and opens arm intake, releasing it closes and stores arm intake
    
    
    //y.whileTrue(new StartEndCommand(() -> {armIntakeSubsystem.unstoreIntake();}, () -> {armIntakeSubsystem.storeIntake();}, armIntakeSubsystem));
    // op rb sets mid height 
    Trigger operatorRB = operatorController.rightBumper();
    operatorRB.onTrue(armSubsystem.setDegPosRefPoint(Arm.HIGHNODE_ANGLE))
    .onFalse(armSubsystem.setBrake());

    Trigger operatorLB = operatorController.leftBumper();
    operatorLB.onTrue(armSubsystem.setDegPosRefPoint(Arm.SHELF_ANGLE))
    .onFalse(armSubsystem.setBrake());
    
    Trigger operatorLT = operatorController.leftTrigger();
    operatorLT.onTrue(armSubsystem.setDegPosRefPoint(Arm.RETRACTED_ANGLE))
    .onFalse(armSubsystem.setBrake());

    Trigger operatorRT = operatorController.rightTrigger();
    operatorRT.onTrue(armSubsystem.setDegPosRefPoint(Arm.FLOOR_ANGLE))
    .onFalse(armSubsystem.setBrake());

    Trigger operatorIntake = operatorController.button(7);
    operatorIntake.onTrue(armIntakeSubsystem.intake())
    .onFalse(armIntakeSubsystem.stop());

    Trigger operatorExtake = operatorController.button(8);
    operatorExtake.onTrue(armIntakeSubsystem.eject());
    /* 
    // raise arm intake at predefined velocity
    Trigger driver3 = driverJoystick.button(3);
    driver3.whileTrue(new StartEndCommand(()->{armIntakeSubsystem.raiseIntake();}, ()->{armIntakeSubsystem.stopWrist();}, armIntakeSubsystem));

    // lower arm intake at predefined velocity
    Trigger driver4 = driverJoystick.button(4);
    driver4.whileTrue(new StartEndCommand(()->{armIntakeSubsystem.lowerIntake();}, ()->{armIntakeSubsystem.stopWrist();}, armIntakeSubsystem));

    // raise arm at set vel
    Trigger driver5 = driverJoystick.button(5);
    driver5.whileTrue(new RepeatCommand(new InstantCommand(()->{armSubsystem.raiseArm();},armSubsystem))).onFalse(armSubsystem.setBrake());
  
    // lower arm at set vel
    Trigger driver6 = driverJoystick.button(6);
    driver6.whileTrue(new RepeatCommand(new InstantCommand(()->{armSubsystem.lowerArm();},armSubsystem))).onFalse(armSubsystem.setBrake());
    //driver6.whileTrue(new InstantCommand(()->{armSubsystem.releaseBrake();},armSubsystem)).onFalse(armSubsystem.setBrake());
    //driver6.whileTrue(new InstantCommand(()->{armSubsystem.lowerArm();},armSubsystem)).onFalse(armSubsystem.setBrake());
    */
  }

  private void setGroundIntakeBindings(){
    // holding x lowers and opens ground intake, releasing it closes and stores
    Trigger operatorX = operatorController.x();
    operatorX.whileTrue(groundIntakeSubsystem.unstoreIntakeCmd())
      .onFalse(groundIntakeSubsystem.storeIntakeCmd());
    // x.whileTrue(new StartEndCommand(() ->
    // {groundIntakeSubsystem.unstoreIntake();}, () ->
    // {groundIntakeSubsystem.storeIntake();}, groundIntakeSubsystem));

    // pressing a ejects game piece for ground intake
    Trigger operatorA = operatorController.a();
    operatorA.whileTrue(groundIntakeSubsystem.dropPiece())
      .onFalse((groundIntakeSubsystem.storeIntakeCmd()));
  }

  private void setLedBindings(){
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

  public void clearArmState(){
    //armSubsystem.stay();
  }

  /*private SequentialCommandGroup autoBalance(){
   return new Balance(drivetrain).withName("Auto Balance")
    .andThen(drivetrain.passiveBrake());
  }*/

  //private SequentialCommandGroup driveOnBalance(){
    //return new InstantCommand(()->{drivetrain.set})
 // }
  //public Command getAutonomousCommand() {
  //  return armCommand;
 // }
}

