// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.commands.auto.testAuto;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.commands.Balance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Com
 * mand-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drivetrain = new SwerveDrive(gyro);
  private final CommandXboxController operatorController = new CommandXboxController(0); 

  // private final CommandXboxController driverController;
  
  private final HashMap<String, Command> eventMap = new HashMap<>();
  private final CommandJoystick driverController;
  
  private final SwerveAutoBuilder auto;

  private final testAuto plsWork = new testAuto(drivetrain);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    // driverController = new CommandXboxController(0);
    auto = new SwerveAutoBuilder(
      drivetrain::getPoseEstimate,
      drivetrain::resetOdometry,
      Constants.Swerve.kDriveKinematics,
      new PIDConstants(0.0, 0.0, 3),
      new PIDConstants(0.0, 0.0, 0.0),
      drivetrain::setModuleStates,
      eventMap,
      true,
      drivetrain
    );
    //driverController = new Joystick(0);
    
    driverController = new CommandJoystick(0);

    // Configure the button bindings
    configureButtonBindings();



  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(new XboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Trigger driverRightBumper = driverController.rightBumper();
    // driverRightBumper.whileTrue(drivetrain.passiveBrake());
    // Trigger driverRightTrigger = driverController.rightTrigger();
    // driverRightTrigger.whileTrue(new RepeatCommand(new InstantCommand(()->drivetrain.normalZeroModules(),drivetrain)));
    Trigger operatorA = operatorController.a();
    //operatorA.whileTrue(m_targetFollowCommand);
  }

  private SequentialCommandGroup autoBalance(){
   return new Balance(drivetrain).withName("Auto Balance")
    .andThen(drivetrain.passiveBrake());
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerTrajectory automaticPath = PathPlanner.loadPath("NicolasValerio Path", 3, 3
    , false);
    
    eventMap.put("marker 1", new PrintCommand("Passed marker 1"));

    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(1, .5), 
    new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)), 
    new PathPoint(new Translation2d(2.0, 0.0), Rotation2d.fromDegrees(-5))//,
    //new PathPoint(new Translation2d(1.99, 1.0), Rotation2d.fromDegrees(0))
    ); 
  
    // 4.7 meters to pick up piece
    FollowPathWithEvents command = new FollowPathWithEvents(
    auto.followPath(automaticPath),
    automaticPath.getMarkers(),
    eventMap
    );
    
    Command manualPath = auto.followPath(traj1);
    Command run = auto.followPath(automaticPath);


    return plsWork;

    // An ExampleCommand will run in autonomous
    
  }
}
