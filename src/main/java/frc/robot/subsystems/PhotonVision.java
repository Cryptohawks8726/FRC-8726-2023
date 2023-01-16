package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

public class PhotonVision {
    
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    // Check if the latest result has any targets.
    if (result.hasTargets()){
        // Get the best target
        var targets = results.getBestTarget();
        // Get information from target
        double yaw = targets.getYaw();
        double pitch = target.getPitch();
        double camToTarget = targets.getCameraToTarget();
    }

}

