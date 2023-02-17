// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;



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
  
  private final Joystick driverController;
  
  private final SwerveAutoBuilder auto;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    
    // driverController = new CommandXboxController(0);
    auto = new SwerveAutoBuilder(
      drivetrain::getPoseEstimate,
      drivetrain::resetOdometry,
      Constants.Swerve.kDriveKinematics,
      new PIDConstants(5.0, 0.0, 0.0),
      new PIDConstants(0.5, 0.0, 0.0),
      drivetrain::setModuleStates,
      eventMap,
      true,
      drivetrain
    );
    //driverController = new Joystick(0);
    
    driverController = new Joystick(0);

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

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("NicolasValerio Path", new PathConstraints(2, 1.5));
    Command run = drivetrain.followTrajectoryCommand(examplePath, true);
    return run;

    // An ExampleCommand will run in autonomous
    
  }
}
