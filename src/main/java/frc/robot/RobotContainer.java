// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmIntake2Subsystem;
import frc.robot.Constants.Arm;
import frc.robot.commands.Balance;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.auto.OneConeBalance;
import frc.robot.commands.auto.OneConeMobility;
import frc.robot.commands.auto.OneConeNoDrive;
import frc.robot.commands.auto.OneCubeMobility;


public class RobotContainer {

  private final PneumaticHub pneumaticHub;
  private final GroundIntakeSubsystem groundIntakeSubsystem;
  private final ArmIntake2Subsystem armIntakeSubsystem;
  private final ArmSubsystem armSubsystem;
  private final SwerveDrive drivetrain;
  private final WristSubsystem wristSubsystem;

  /*private final OneConeMobility OneConeAuto;
  private final OneCubeMobility OneCubeAuto;
  private final OneConeNoDrive OneConeNoDrive;
  private OneConeBalance oneConeBalance;*/

  private final LED ledStrip = new LED(Constants.LED_PORT, Constants.LED_LENGTH);

  private CommandXboxController operatorController;
  private CommandJoystick driverJoystick;
  private SendableChooser<Command> autoChooser;
  private SendableChooser<Boolean> posChooser;
  private UsbCamera driveCam;
  private MjpegServer camServ;



  public RobotContainer() {
    drivetrain = new SwerveDrive();
    armIntakeSubsystem = new ArmIntake2Subsystem();
    pneumaticHub = new PneumaticHub(Constants.COMPRESSOR_ID);
    // /pneumaticHub.disableCompressor();
    pneumaticHub.enableCompressorDigital();
    groundIntakeSubsystem = new GroundIntakeSubsystem();
    armSubsystem = new ArmSubsystem();
    wristSubsystem = new WristSubsystem();
    driverJoystick = new CommandJoystick(Constants.DRIVER_CONTROLLER);
    operatorController = new CommandXboxController(Constants.OPERATOR_XBOX);
    //oneConeBalance = new OneConeBalance(drivetrain, armIntakeSubsystem, wristSubsystem, armSubsystem, false);
    //OneConeAuto = new OneConeMobility(drivetrain,armIntakeSubsystem,wristSubsystem,armSubsystem, false);
    //OneCubeAuto = new OneCubeMobility(drivetrain,armIntakeSubsystem,wristSubsystem,armSubsystem, false);
    //OneConeNoDrive = new OneConeNoDrive(drivetrain, armIntakeSubsystem, wristSubsystem, armSubsystem, false);
    posChooser = new SendableChooser<Boolean>();
    autoChooser = new SendableChooser<Command>();
   /* autoChooser.addOption("new One Cube Blue",autos.oneCubeMobilityCmd(true));
    autoChooser.addOption("new one cube not blue ", autos.oneCubeMobilityCmd(false));
    autoChooser.addOption("new one cone blue", autos.oneConeMobilityCmd(true));
    autoChooser.addOption("new one cone not blue", autos.oneConeMobilityCmd(false));*/
    autoChooser.addOption("One Cube BlueShelf",new OneCubeMobility(drivetrain,armIntakeSubsystem,wristSubsystem,armSubsystem, true));
    autoChooser.addOption("One Cube Not BlueShelf",new OneCubeMobility(drivetrain,armIntakeSubsystem,wristSubsystem,armSubsystem, false));
    autoChooser.addOption("One Cone BlueShelf",new OneConeMobility(drivetrain,armIntakeSubsystem,wristSubsystem,armSubsystem, true));
    autoChooser.addOption("One Cone Not BlueShelf",new OneConeMobility(drivetrain,armIntakeSubsystem,wristSubsystem,armSubsystem, false));
    autoChooser.addOption("Cone Balance", new OneConeBalance(drivetrain, armIntakeSubsystem, wristSubsystem, armSubsystem, false));
    autoChooser.setDefaultOption("One Cone No Drive", new OneConeNoDrive(drivetrain, armIntakeSubsystem, wristSubsystem, armSubsystem, false));
    SmartDashboard.putData(autoChooser);
    //driveCam = new UsbCamera("Drive", 1);
    driveCam = CameraServer.startAutomaticCapture();
    driveCam.setResolution(160, 120);
 
    //camServ = new MjpegServer("drive cam", 1181);
    //camServ.setSource(driveCam);
    //driveCam.setResolution(320, 160);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //drive cmds
    drivetrain.setDefaultCommand(new XboxTeleopDrive(drivetrain,driverJoystick).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    setArmBindings();
    setGroundIntakeBindings();
    setLedBindings();

  }

  private void setArmBindings(){
    
    Trigger operatorDown = operatorController.povDown();
    operatorDown.onTrue(new InstantCommand(()->{
      armSubsystem.setGoal(Arm.FLOOR_ANGLE,groundIntakeSubsystem.isExtended)
      ;}));
    
    Trigger operatorLeft = operatorController.povLeft();
    operatorLeft.onTrue(new InstantCommand(()->{
      armSubsystem.setGoal(Arm.MID_CUBE_ANGLE,groundIntakeSubsystem.isExtended)
      ;}));
    
    Trigger operatorRight = operatorController.povRight();
    operatorRight.onTrue(new InstantCommand(()->{
      armSubsystem.setGoal(Arm.MID_ANGLE,groundIntakeSubsystem.isExtended)
      ;}));
    
    Trigger opUp = operatorController.povUp();
    opUp.onTrue(new InstantCommand(()->{
      armSubsystem.setGoal(Arm.HIGHNODE_ANGLE,groundIntakeSubsystem.isExtended)
      ;}));

    Trigger operatorRT = operatorController.rightTrigger();
    operatorRT.onTrue(new InstantCommand(()->{
      armSubsystem.setGoal(Arm.SHELF_CONE,groundIntakeSubsystem.isExtended) //cone
      ;}));

    Trigger operatorLT = operatorController.leftTrigger();
    operatorLT.onTrue(new InstantCommand(()->{
      armSubsystem.setGoal(Arm.SHELF_CUBE,groundIntakeSubsystem.isExtended) //cube 
      ;}));

    Trigger operatorIntake = operatorController.button(7);
    operatorIntake.onTrue(armIntakeSubsystem.eject())
    .onFalse(armIntakeSubsystem.stop());

    Trigger operatorExtake = operatorController.button(8);
    operatorExtake.onTrue(armIntakeSubsystem.intake())
    .onFalse(armIntakeSubsystem.stop());


    Trigger operatorLB = operatorController.leftBumper();
    operatorLB.onTrue(new InstantCommand(()->{wristSubsystem.shelfExtend();})); //cube

    Trigger operatorRB = operatorController.rightBumper();
    operatorRB.onTrue(new InstantCommand(()->{wristSubsystem.toggleExtend();}));

    Trigger operatorY = operatorController.y();
    operatorY.onTrue(new InstantCommand(() -> {
      armSubsystem.setGoal(Arm.RETRACTED_ANGLE,groundIntakeSubsystem.isExtended);
    })
        .andThen(new InstantCommand(() -> {
          wristSubsystem.retractWrist();
        })));

      Trigger operatorB = operatorController.b();
      operatorB.onTrue(new InstantCommand(()->{armSubsystem.setGoal(Arm.SUBSTATION_CUBE_ANGLE,groundIntakeSubsystem.isExtended);}));
  }

  private void setGroundIntakeBindings(){
    // holding x lowers and opens ground intake, releasing it closes and stores
    Trigger operatorX = operatorController.x();
    operatorX.whileTrue(groundIntakeSubsystem.unstoreIntakeCmd())
      .onFalse(groundIntakeSubsystem.storeIntakeCmd());
    /*
    Trigger operatorX = operatorController.x();
    operatorX.onTrue(
      groundIntakeSubsystem.isExtended ? new InstantCommand(()->{groundIntakeSubsystem.raiseIntake;})
      :new InstantCommand(()->{groundIntakeSubsystem.lowerIntake;});

      Trigger operatorA = operatorController.a();

     */

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
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
 }
}

