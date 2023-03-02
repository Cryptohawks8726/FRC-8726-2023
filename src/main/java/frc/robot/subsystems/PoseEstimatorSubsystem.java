//package frc.robot.subsystems;
package frc.robot.subsystems;

import static frc.robot.Constants.PhotonVisionConstants.APRILTAG_CAMERA_TO_ROBOT;
import static frc.robot.Constants.PhotonVisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.PhotonVisionConstants.FIELD_WIDTH_METERS;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.SwerveDrive;



public class PoseEstimatorSubsystem extends SubsystemBase {

    
  
    public PhotonCamera photonCamera = new PhotonCamera("Spinel1");
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveKinematics kinematics;
    private SwerveDrive drivetrain;
    private PhotonPoseEstimator photonPoseEstimator;

    private double previousPipelineTimestamp = 0;
    private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;

    private boolean sawTag = false;
    private final Field2d field2d = new Field2d();
   
    public PoseEstimatorSubsystem(PhotonCamera photonCamera, SwerveDrive drivetrain) {
        this.drivetrain = drivetrain;
        PhotonPoseEstimator photonPoseEstimator = null;
        try {
            AprilTagFieldLayout layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            layout.setOrigin(originPosition);
            if (photonCamera != null) {
              photonPoseEstimator =
                  new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, photonCamera, APRILTAG_CAMERA_TO_ROBOT);
            }
          } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
          }
          this.photonPoseEstimator = photonPoseEstimator;

        poseEstimator = SwerveDrive.getPoseEstimator();

          

    }

    /*public void addDashboardWidgets(ShuffleboardTab tab) {
      //tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
     // tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
      tab.add("Field", field2d);
      tab.addString("Pose", this::getFomattedPose);
      

    }
    /*
    
    /**
    * Sets the alliance. This is used to configure the origin of the AprilTag map
    * @param alliance alliance
    */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;
        switch(alliance) {
            case Blue:
            allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
            originPosition = OriginPosition.kBlueAllianceWallRightSide;
            break;
            case Red:
            allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
            originPosition = OriginPosition.kRedAllianceWallRightSide;
            break;
            default:
            // No valid alliance data. Nothing we can do about it
        }
        if (photonPoseEstimator != null) {
            photonPoseEstimator.getFieldTags().setOrigin(originPosition);
        }
        if (allianceChanged && sawTag) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
            // needs to be transformed to the new coordinate system.
            var newPose = flipAlliance(poseEstimator.getEstimatedPosition());
            setCurrentPose(newPose);
        }
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //poseEstimator.update(drivetrain.getRobotAngle(), drivetrain.getSwerveModulePositions());
    PhotonTrackedTarget lowestAmbiguityTarget = null;
    var result = photonCamera.getLatestResult();
    double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : result.targets) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            System.out.println(targetPoseAmbiguity);
            // Make sure the target is a Fiducial target.
            if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = targetPoseAmbiguity;
                lowestAmbiguityTarget = target;
                System.out.println("yay");
            }
        }

    if (photonPoseEstimator != null) {
        //System.out.println("IDK");
      // Update pose estimator with the best visible target
        Optional<EstimatedRobotPose> estPose = photonPoseEstimator.update(photonCamera.getLatestResult());
        estPose.ifPresent((estimatedRobotPose -> {
          //System.out.println("Maybe this works");
          //sawTag = true;
        //var estimatedPose = estimatedRobotPose.estimatedPose;
        // Make sure we have a new measurement, and that it's on the field
        /*if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp
            && estimatedPose.getX() > 0.0 && estimatedPose.getX() <= PhotonVisionConstants.FIELD_LENGTH_METERS
            && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= PhotonVisionConstants.FIELD_WIDTH_METERS) {*/
            //previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
            getFormattedPose(estimatedRobotPose);
           // poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
           // }
        }));
    } 
    
    Pose2d dashboardPose = getCurrentPose();
    if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);

    //getFormattedPose();

  }

  private String getFormattedPose(EstimatedRobotPose estPose) {
    //var pose = getCurrentPose();    
    Pose2d pose = estPose.estimatedPose.toPose2d();
   
    String values = String.format("(%.3f, %.3f) %.2f degrees", 
    pose.getX(), 
    pose.getY(),
    pose.getRotation().getDegrees());

    SmartDashboard.putString("X,Y, Rotational", values);

    return values;
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
    
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      drivetrain.getRobotAngle(),
      drivetrain.getSwerveModulePositions(),
      newPose);
  }

  /**
  * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
  * what "forward" is for field oriented driving.
  */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(new Pose2d(
      new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
      new Rotation2d(Math.PI)));
  }
}

