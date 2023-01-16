package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase{

    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    XboxController xboxController;

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    public PhotonVision(){

        if (xboxController.getAButton()) {
            // Query the latest result from PhotonVision
            PhotonPipelineResult result = camera.getLatestResult();

            if (result.hasTargets()){
                double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS, 
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
                System.out.println(range);
            }
        }
    }
}
