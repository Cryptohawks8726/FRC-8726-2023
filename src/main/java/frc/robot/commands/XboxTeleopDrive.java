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
    private final Joystick controller;
    
    private final SwerveDrive drivetrain;
    private Rotation2d lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;
    private boolean isRobotRelative=false;


    // TODO: Implement Optimization
    
    public XboxTeleopDrive(SwerveDrive drivetrain, /*CommandXboxController controller*/ Joystick controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
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
        /* X axis is forward from driver perspective, and the Y axis is parallel to the driver station wall. 
    See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html 
        The controller axes have x as left-right and y as up-down
        */
        // boolean isRobotRelative = controller.leftBumper().getAsBoolean();

        if(controller.getTriggerPressed()){
            isRobotRelative = !isRobotRelative;
        }   

        boolean theta_user_set = false;


        
        // Get Controller Values
        // FIXME: xVel and yVel pull the wrong values? Flip them
        // double xVel = (Math.abs(controller.getLeftY()) > 0.1 ? controller.getLeftY() : 0.0); 
        // double yVel = (Math.abs(controller.getLeftX()) > 0.1 ? controller.getLeftX() : 0.0);
        // double thetaVel = (Math.abs(controller.getRightX()) > 0.1 ? controller.getRightX() * Constants.Swerve.maxAngularSpeed : 0.0);
        double thetaVel=0;
        double xVel = (Math.abs(controller.getX()) > 0.1 ? controller.getX() : 0.0); 
        double yVel = (Math.abs(controller.getY()) > 0.1 ? controller.getY() : 0.0);        
        thetaVel = (Math.abs(controller.getZ()) > 0.1 ? controller.getZ() * Constants.Swerve.maxAngularSpeed : 0.0);
        
        if(thetaVel != 0){
            theta_user_set =true;
        }

        double sensitivity = (controller.getThrottle()*-1)+1.01;
        xVel = -Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed * sensitivity; //square input while preserving sign
        yVel = Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed * sensitivity;

        // maintain heading if there's no rotational input
        if (!theta_user_set){
            //if (isHeadingSet == false){
                    headingPID.reset();
                    isHeadingSet = true;
                    lastHeading = drivetrain.getRobotAngle();
                    headingPID.setSetpoint(lastHeading.getDegrees()%180);
                    thetaVel = headingPID.calculate(drivetrain.getRobotAngle().getDegrees()%180);
            //}else{
                    
            //}
        }


        if(!isRobotRelative){
            Rotation2d rotation = drivetrain.getRobotAngle();
            ChassisSpeeds speed_fieldRel = new ChassisSpeeds(
                xVel * rotation.getCos() - yVel * rotation.getSin(),
                xVel * rotation.getSin() + yVel * rotation.getCos(),
                thetaVel);
            drivetrain.drive(speed_fieldRel, true);
        } else {
            drivetrain.drive(new ChassisSpeeds(xVel, yVel, thetaVel),true);
        }
    }

}
