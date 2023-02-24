package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.RobotPoseEstimator;
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

public class PhotonVision extends SubsystemBase{

    // Change this to match the name of your camera
    public PhotonCamera camera = new PhotonCamera("Spinel1");
    private AprilTagFieldLayout tagMap;
    private AHRS gyro;
    private RobotPoseEstimator poseEstimator;
    private Transform3d robotToCam;
    private SwerveDrivePoseEstimator swerveEstimator;

    public PhotonVision(AHRS ahrsGyro, Transform3d robotToCam, SwerveDrivePoseEstimator swerveEstimator){
        robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(2),0.0,PhotonVisionConstants.CAMERA_HEIGHT_METERS),new Rotation3d());
        this.swerveEstimator = swerveEstimator;
        //poseEstimator = new RobotPoseEstimator(tagMap, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, new List<Pair>(new Pair(camera,robotToCam));
        gyro = ahrsGyro;
        gyro.calibrate(); // possibly move to avoid the robot being moved during calibration
        gyro.reset();
    }
    
    public void periodic(){  
        SmartDashboard.putNumber("Target Yaw", getYaw());
        System.out.println(getPhotonPoseEstimator2d());
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


    public Pose2d getPhotonPoseEstimator2d(){
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult(); 

        PhotonTrackedTarget target = result.getBestTarget();

        try {
            tagMap = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
       } catch (Exception e){
       } 

        // Get the current best target.
        //Transform3d cameraToTarget = target.getBestCameraToTarget();
        
        // Default rotation of Pose2d
        Rotation2d default2dPose_rotation = new Rotation2d(0);

        // Final rotation of Pose2d
        Rotation2d final2dPose_rotation = new Rotation2d(45);

        // Error rotation of Pose2d
        Rotation2d error2dPose_rotation = new Rotation2d(-1);

        Pose2d defaultPose2d = new Pose2d(PhotonVisionConstants.default2dPose_X,
            PhotonVisionConstants.default2dPose_Y, default2dPose_rotation);

        Pose2d finalPose2d = new Pose2d(PhotonVisionConstants.final2dPose_X,
            PhotonVisionConstants.final2dPose_Y, final2dPose_rotation);

        Pose2d errorPose2d = new Pose2d(PhotonVisionConstants.error2dPose_X,
            PhotonVisionConstants.error2dPose_Y, error2dPose_rotation);
  

        if (result.hasTargets()){ 
            //Pose2d targetPose = new Pose2d(cameraToTarget.getTranslation().toTranslation2d(), cameraToTarget.getRotation().toRotation2d());
            Pose2d fieldToTarget = tagMap.getTagPose(target.getFiducialId()).get().toPose2d();

            Transform2d cameraToRobot = new Transform2d(defaultPose2d, finalPose2d);

            // Calculate robot's field relative pose

            Pose2d robotPose = PhotonUtils.estimateFieldToRobot(PhotonVisionConstants.kCameraHeight,
                PhotonVisionConstants.kTargetHeight, PhotonVisionConstants.kCameraPitch, PhotonVisionConstants.kTargetPitch,
                Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), fieldToTarget, cameraToRobot);

            return  robotPose;
        } else {
            return errorPose2d;
        }
    }
     
    public Rotation2d robotHeading(){
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult(); 
        PhotonTrackedTarget target = result.getBestTarget();

        Rotation2d errorYaw = new Rotation2d(-1);

        Transform3d targetPose = target.getBestCameraToTarget();

        Rotation2d targetYaw = PhotonUtils.getYawToPose(swerveEstimator.getEstimatedPosition(), new Pose2d(targetPose.getTranslation().toTranslation2d(),targetPose.getRotation().toRotation2d()));

        if (result.hasTargets()){
            return targetYaw;
        } else {
            return errorYaw;
        }
    }

    public double getYaw(){
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult(); 
        PhotonTrackedTarget target = result.getBestTarget();

        double yaw = -1;
        
        if (result.hasTargets()){
            yaw = target.getYaw();
        }
        return yaw;
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