package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TargetFollowCommand;

public class PhotonVision extends SubsystemBase{

    // Constants such as camera and target height stored. Change per robot and goal!
    // Actual values will be dependent on final bot
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(6.58);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(3);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(181.451);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);


    // Change this to match the name of your camera
    public PhotonCamera camera = new PhotonCamera("Spinel1");

    public PhotonVision(){
    }
    
    public void periodic(){ 
        
    }



    @Override 
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public double getDistance(){
        System.out.println("photonvison subsystem get Distance");
        var result = camera.getLatestResult();
        double range = -1;
        if (result.hasTargets()){
            range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS, 
            Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return range;
    }

    
}
