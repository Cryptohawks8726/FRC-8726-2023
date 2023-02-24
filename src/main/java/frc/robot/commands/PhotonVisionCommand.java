package frc.robot.commands;

import static frc.robot.Constants.Swerve.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.Swerve.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.Swerve.maxAngularSpeed;

import static frc.robot.Constants.Swerve.kPXController;
import static frc.robot.Constants.Swerve.kPYController;
import static frc.robot.Constants.Swerve.kPThetaController;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class PhotonVisionCommand extends CommandBase{

  private static final double TRANSLATION_TOLERANCE = 0.2;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(0.5);

  private static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    kMaxSpeedMetersPerSecond * .9,
    kMaxAccelerationMetersPerSecondSquared * 3.0);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    maxAngularSpeed* 0.9,
    maxAngularSpeed* 3.0);

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  private SwerveDrive drivetrain;
  private Supplier<Pose2d> poseProvider;
  private boolean endAtGoal;
  private Pose2d goalPose;


  public PhotonVisionCommand(
      SwerveDrive drivetrain,
      Supplier<Pose2d> poseProvider,
      Pose2d goalPose) {
    this(drivetrain, poseProvider, goalPose, true, XY_CONSTRAINTS, OMEGA_CONSTRAINTS);
  }

  public PhotonVisionCommand(
    SwerveDrive drivetrain,
    Supplier<Pose2d> poseProvider,
    Pose2d goalPose,
    boolean endAtGoal,
    TrapezoidProfile.Constraints xyConstraints,
    TrapezoidProfile.Constraints omegaConstraints) {
    this.drivetrain = drivetrain;
    this.poseProvider = poseProvider;
    this.endAtGoal = endAtGoal;
    this.goalPose = goalPose;

    xController = new ProfiledPIDController(kPXController, kPYController, kPThetaController, xyConstraints);
    yController = new ProfiledPIDController(kPXController, kPYController, kPThetaController, xyConstraints);
    thetaController = new ProfiledPIDController(kPXController, kPYController, kPThetaController, xyConstraints);

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
}
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PhotonVisionCommand initialize");
    resetPIDControllers();

    thetaController.setTolerance(THETA_TOLERANCE);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);

    setGoalPose(goalPose);
  }

  public void setGoalPose(Pose2d goalPose) {
    if (goalPose != null) {
      thetaController.setGoal(goalPose.getRotation().getRadians());
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
    }
    this.goalPose = goalPose;
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("PhotonVisionCommand execute");
    var robotPose = poseProvider.get();
    if (goalPose == null) {
      // TODO use slew rate limiter to just stop the robot
      // This might be crazy, just set the goal to the current pose. It _should_ plan to decelerate and then come back
      // but updating the goal on each iteration should just decelerate until we reach the goal
      xController.setGoal(robotPose.getX());
      yController.setGoal(robotPose.getY());
      thetaController.setGoal(robotPose.getRotation().getRadians());
    }
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    // TODO check if the "isClosedLoop" is ok
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()), true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endAtGoal && atGoal();  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.passiveBrake();
  }
}

