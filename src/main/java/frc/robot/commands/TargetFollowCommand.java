package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveDrive;

public class TargetFollowCommand extends CommandBase{
    private PhotonVision m_photonVisionSubsystem;
    private XboxController xbox; 
    private SwerveDrive drivetrain;

	public TargetFollowCommand(PhotonVision subsystem, XboxController controller, SwerveDrive code) {
        m_photonVisionSubsystem = subsystem;
        xbox = controller;
        drivetrain = code;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_photonVisionSubsystem);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        System.out.println("intialized");
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        System.out.println("working");
        //config
        if(xbox.getAButton() && m_photonVisionSubsystem.getDistance()!=-1){
            TrajectoryConfig config = new TrajectoryConfig(Constants.Swerve.kMaxSpeedMetersPerSecond, 
            Constants.Swerve.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.kDriveKinematics);
            
            /* 
            double initialx = 0;
            double intialy = 0;
            double initialangle = m_photonVisionSubsystem.camera.getLatestResult().getBestTarget().getYaw();

            double finalx = 

            double finaly = 
            double finalangle = 
            */


            //actual path to follow
            Trajectory path1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), 
            List.of(),  
            new Pose2d(3, 0, new Rotation2d(0)), config);


            //tracking path

            PIDController xController = new PIDController(Constants.Swerve.kPXController, 0, 0);
            PIDController yController = new PIDController(Constants.Swerve.kPYController, 0, 0);
            ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Swerve.kPThetaController, 0, 0, Constants.Swerve.kThetaControllerConstraints);

            thetaController.enableContinuousInput(-Math.PI, Math.PI);


            //command to follow path
            SwerveControllerCommand swervepath = new SwerveControllerCommand(
            path1, 
            drivetrain::getPoseEstimate,
            Constants.Swerve.kDriveKinematics,
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);
            
            swervepath.execute();
            //new InstantCommand(()->drivetrain.resetOdometry(path1.getInitialPose()));


        }else{
            System.out.println("not working");
        }
                
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {}
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
