package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.lang.Math;

public class XboxTeleopDrive extends CommandBase{
    // private final CommandXboxController controller;

    // Initialize Variables
    private final Joystick controller;
    private final SwerveDrive drivetrain;
    private Rotation2d lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;
    private boolean isRobotRelative=false; // Initial State of Robot

    
    public XboxTeleopDrive(SwerveDrive drivetrain, /*CommandXboxController controller*/ Joystick controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);

        // Create Heading PID
        headingPID = new PIDController(Constants.Swerve.kHeadingP, Constants.Swerve.kHeadingI, Constants.Swerve.kHeadingD, 20);
        headingPID.enableContinuousInput(0, 360);
    }


    @Override
    public void initialize(){
        isHeadingSet = false;
        headingPID.reset();
    } 
     
    @Override
    public void execute(){
        /* X axis is forward from driver perspective, and the Y axis is parallel to the driver station wall. */

        
        // Change from Robot Relative to Field Relative
        if(controller.getTriggerPressed()){
            isRobotRelative = !isRobotRelative;
        }   

        // Is Robot Rotating?
        boolean theta_user_set = false;


        
        // Get Controller Values
        double thetaVel=0; // Set Rotation to 0
        double xVel = (Math.abs(controller.getX()) > 0.1 ? controller.getX() : 0.0);    // Get X
        double yVel = (Math.abs(controller.getY()) > 0.1 ? controller.getY() : 0.0);    // Get Y   
        thetaVel = (Math.abs(controller.getZ()) > 0.1 ? controller.getZ() * Constants.Swerve.maxAngularSpeed : 0.0);    // Get Rotation Axis
        
        // If there is no rotation, theta_user_set is false
        if(thetaVel != 0){
            theta_user_set =true;
        }

        // Set Sensitivity of Robot Speed through the slider
        double sensitivity = (controller.getThrottle()*-1)+1.01;

        // Calculate velocities
        xVel = -Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed * sensitivity; //square input while preserving sign
        yVel = Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed * sensitivity;

        // maintain heading if there's no rotational input
        if (!theta_user_set){
            headingPID.reset();
            isHeadingSet = true;
            lastHeading = drivetrain.getRobotAngle();
            headingPID.setSetpoint(lastHeading.getDegrees()%180);
            thetaVel = headingPID.calculate(drivetrain.getRobotAngle().getDegrees()%180);
        }

        // Field vs Robot Relative 
        if(!isRobotRelative){   // Field Relative
            // Get Robot Angle
            Rotation2d rotation = drivetrain.getRobotAngle();

            // Calculate New Chassis Speeds
            ChassisSpeeds speed_fieldRel = new ChassisSpeeds(
                xVel * rotation.getCos() - yVel * rotation.getSin(),
                xVel * rotation.getSin() + yVel * rotation.getCos(),
                thetaVel);

            // Drive
            drivetrain.drive(speed_fieldRel, true);
        }
        else {
            // Drive Robot Relative
            drivetrain.drive(new ChassisSpeeds(xVel, yVel, thetaVel),true);
        }
    }

}
