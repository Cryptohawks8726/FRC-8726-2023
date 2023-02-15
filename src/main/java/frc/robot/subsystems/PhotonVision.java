package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TargetFollowCommand;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.SwerveDrive;

public class PhotonVision extends SubsystemBase{

    // Change this to match the name of your camera
    public PhotonCamera camera = new PhotonCamera("Spinel1");

    private AHRS gyro;

    public PhotonVision(AHRS ahrsGyro){
        gyro = ahrsGyro;
        gyro.calibrate(); // possibly move to avoid the robot being moved during calibration
        gyro.reset();
    }
    
    public void periodic(){ 
        System.out.println("This is Pose2d: " + getPhotonPoseEstimator());   
    }

    public double getDistance(){
        System.out.println("photonvison subsystem get Distance");
        var result = camera.getLatestResult();
        double range = -1;
        if (result.hasTargets()){
            range = PhotonUtils.calculateDistanceToTargetMeters(PhotonVisionConstants.CAMERA_HEIGHT_METERS,
            PhotonVisionConstants.TARGET_HEIGHT_METERS,
            PhotonVisionConstants.CAMERA_PITCH_RADIANS, 
            Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return range;
    }

    public Pose2d getPhotonPoseEstimator(){
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();        

        // Get the current best target.
        PhotonTrackedTarget target = result.getBestTarget();

        // Default rotation of Pose2d
        Rotation2d default2dPose_rotation = new Rotation2d(0);

        Pose2d defaultPose2d = new Pose2d(PhotonVisionConstants.default2dPose_X,
            PhotonVisionConstants.default2dPose_Y, default2dPose_rotation);

        if (result.hasTargets()){
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Pose2d targetPose = new Pose2d(cameraToTarget.getTranslation().toTranslation2d(), cameraToTarget.getRotation().toRotation2d());

            // probably needs to be changed
            Transform2d cameraToRobot = new Transform2d(targetPose, targetPose);

            // Calculate robot's field relative pose
            // cameraToRobot: The position of the robot relative to the camera. If the camera was mounted 3 inches behind
            //   the "origin" (usually physical center) of the robot, this would be Transform2d(3 inches, 0 inches, 0 degrees).
            Pose2d robotPose = PhotonUtils.estimateFieldToRobot(PhotonVisionConstants.kCameraHeight, PhotonVisionConstants.kTargetHeight,
                PhotonVisionConstants.kCameraPitch, PhotonVisionConstants.kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()),
                gyro.getRotation2d(), targetPose, cameraToRobot);

            return  robotPose;
        } else {
            return defaultPose2d;
        }



        /* 
        // The parameter for loadFromResource() will be different depending on the game.
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        
        //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        //These values will need to be changed by robot, should add to constants when finalized
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), 
            new Rotation3d(0,0,0)); 

        // Calculate robot's field relative pose
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), 
            aprilTagFieldLayout.getTagPose(target.getFiducialId()), 0);
        */
        
    }
    
}