package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

public class PhotonVision {
    
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    
}
