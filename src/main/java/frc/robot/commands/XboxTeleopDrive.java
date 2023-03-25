package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.lang.Math;

public class XboxTeleopDrive extends CommandBase{
    // private final CommandXboxController controller;
    private final CommandJoystick controller;
    
    private final SwerveDrive drivetrain;
    private double lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;
    
    public XboxTeleopDrive(SwerveDrive drivetrain, CommandJoystick controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
        headingPID = new PIDController(/*Constants.Swerve.kHeadingP*/0.04, Constants.Swerve.kHeadingI, Constants.Swerve.kHeadingD, 20);
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

        boolean isRobotRelative = controller.trigger().getAsBoolean();
        
        double xVel = (Math.abs(controller.getY()) > 0.2 ? -controller.getY() : 0.0); 
        double yVel = (Math.abs(controller.getX()) > 0.2 ? -controller.getX() : 0.0);        
        double thetaVel = (Math.abs(controller.getZ()) > 0.3 ? controller.getZ() : 0.0);
        double sensitivity = Math.pow((controller.getThrottle()*-1)+1.01, 2);
        xVel = Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed * sensitivity; //square input while preserving sign
        yVel = Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed * sensitivity;
        thetaVel = Math.signum(thetaVel)* Math.pow(thetaVel,2) * Constants.Swerve.maxAngularSpeed *sensitivity;
        
        if(controller.top().getAsBoolean()){
            thetaVel = 0.0;
            double ang = drivetrain.gyro.getYaw()+180%360;
            if(ang<90.0 || ang > 270.0){
                lastHeading = 0.0;
            }
            else{
                lastHeading = 180.0;
                
            }
            isHeadingSet = true;
            yVel = 0.0;
            headingPID.setSetpoint(lastHeading);
        }
        
        // maintain heading if there's no rotational input
         if (Math.abs(thetaVel) < 0.3 && ((Math.abs(xVel)>0.2) ||(Math.abs(yVel)>0.2))){
            if (isHeadingSet == false){
                headingPID.reset();
                isHeadingSet = true;
                lastHeading = drivetrain.gyro.getYaw()+180%360;
                headingPID.setSetpoint(lastHeading);
                thetaVel = headingPID.calculate(drivetrain.gyro.getYaw()+180%360);
                SmartDashboard.putNumber("calc theta",thetaVel);
                //thetaVel = 0.0;
             }else{
                thetaVel = headingPID.calculate(drivetrain.gyro.getYaw()+180%360);
                SmartDashboard.putNumber("calc theta",thetaVel);
                //thetaVel = 0.0;
             }
         } else{
            isHeadingSet = false;
         }
         SmartDashboard.putNumber("set heading",lastHeading);
         SmartDashboard.putNumber("curr heading",drivetrain.gyro.getYaw()+180%360);
        drivetrain.drive(
             isRobotRelative ? new ChassisSpeeds(xVel, yVel, thetaVel)
            : ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, drivetrain.getRobotAngle())
            ,true
        );
    }

}
