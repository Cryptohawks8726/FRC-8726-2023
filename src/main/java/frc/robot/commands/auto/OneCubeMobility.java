package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.ArmIntake;
import frc.robot.subsystems.ArmIntake2Subsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.WristSubsystem;

// 35 inches from nodes

public class OneCubeMobility extends CommandBase {
    private SwerveDrive driveTrain;

    private boolean flag = true;
    private boolean flag1 = true;
    private boolean flag2 = true;
    private boolean flag3 = true;
    private boolean armRaised = true;
    private boolean armStarted = false;
    private boolean armUp = false;
    private boolean isBlueShelf;
    
    private ArmIntake2Subsystem armIntake;
    private WristSubsystem wrist;
    private ArmSubsystem arm;
    private Timer timer = new Timer();

    public OneCubeMobility(SwerveDrive swerve, ArmIntake2Subsystem armIntake, WristSubsystem wrist, ArmSubsystem arm, boolean blueShelf) {
        driveTrain = swerve;
        this.armIntake = armIntake;
        this.arm = arm;
        this.wrist = wrist;

        isBlueShelf = blueShelf;

        addRequirements(swerve);
        addRequirements(arm);
        addRequirements(wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveTrain.resetOdometry(new Pose2d());
       flag = true;
        flag1 = true;
        flag2 = true;
      flag3 = true;
        armRaised = true;
         armStarted = false;
    armUp = false;
    
        //driveTrain.gyro.reset();
    }
    @Override
    public void execute() {
        // driveTrain.logValues();
        driveTrain.odometry.update(
             driveTrain.gyro.getRotation2d(), 
            driveTrain.getSwerveModulePositions()
        );
        if(armRaised){
            arm.setGoal(Arm.MID_ANGLE + 7.5,false);
            wrist.shelfExtend();
            if(armStarted == false){
                timer.start();
                armStarted=true;
            }
        }else{
            arm.setGoal(Arm.RETRACTED_ANGLE,false);
            wrist.retractWrist();
        }
        if(armRaised && armStarted && timer.get() > 2.5){
            armUp = true;
        }
  
        if(flag1 && armUp && setPosition(-0.9, 0.0, 0.0, 0.8, 0.5, 0.5, 0.05, 0.05, 5.0)) {
            flag1 = false;
            timer.reset();
            timer.start(); 
         } 

         if(!flag1 && flag2 && timer.get() > 0.25){
            armIntake.intake().schedule();
            if(timer.get() > 1.5){
                armIntake.stop().schedule();
                double yPos = isBlueShelf ? 0.205 : -0.205;
                if (setPosition(3.5, yPos, 0.0, 2.0, 0.5, 0.5, 0.05, 0.1, 3.0)) {
                    flag2 = false;
                }
                if (timer.get() > 3.0) {
                    armRaised = false;
                }
            }
         }
         if(!flag2){
            armRaised = false;
         }
         


        


        //cone
        //swing arm up and then outtake
        //new while block, drive backwards short distance, drop arm
        //new while block, drive backwards to pass community zone

        //cube
        //swing arm up
        //new while loop, drive forwards 16 inch, outtake
        //new while block, drive backwards short distance, drop arm
        //new while block, drive backwards to pass community zone ~110 inches

        // if(flag && !(flag1)) {
        //     driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true);
        //     flag = false;
        // }

    }  
    
    public boolean setPosition(double xPos, double yPos, double thetaPos, double xSpeed, double ySpeed, double thetaSpeed, double xTolerance, double yTolerance, double thetaTolerance) {
        double xVel = 0.0;
        double yVel = 0.0;
        double thetaVel = 0.0;
        boolean x = false;
        boolean y = false;
        boolean theta = false;

        if (xPos > driveTrain.getPoseEstimate().getX() + xTolerance) {
            xVel = xSpeed;
            x = false;
        } else if (xPos < driveTrain.getPoseEstimate().getX() - xTolerance){
            xVel = -xSpeed;
            x = false;
        } else {
            xVel = 0.0;
            x = true;
        }
       
        if (yPos > driveTrain.getPoseEstimate().getY() + yTolerance) {
            yVel = ySpeed;
            y = false;
        } else if (yPos < driveTrain.getPoseEstimate().getY() - yTolerance){
            yVel = -ySpeed;
            y = false;
        } else {
            yVel = 0.0;
            y = true;
        }

         if (thetaPos > driveTrain.getPoseEstimate().getRotation().getDegrees() + thetaTolerance) {
             thetaVel = -thetaSpeed;
             theta = false;
         } else if (thetaPos < driveTrain.getPoseEstimate().getRotation().getDegrees() - thetaTolerance){
             thetaVel = thetaSpeed;
             theta = false;
         } else {
             thetaVel = 0.0;
             theta = true;
         }
 
        driveTrain.drive(new ChassisSpeeds(xVel, yVel, thetaVel), true);

        return x && y && theta;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true);
    }
}