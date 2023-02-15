package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVision;

public class PhotonVisionCommand extends CommandBase{

  private PhotonVision m_photonVisionSubsystem;

	public PhotonVisionCommand(PhotonVision subsystem) {
    m_photonVisionSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_photonVisionSubsystem);

    }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        System.out.println("PhotonVisionCommand initialize");
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        System.out.println("PhotonVisionCommand execute");
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
