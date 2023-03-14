package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;


public class testAuto extends CommandBase {
    private SwerveDrive driveTrain;

    private boolean flag = true;
    private boolean flag1 = true;
    private boolean flag2 = true;
    private boolean flag3 = true;

    public testAuto(SwerveDrive swerve) {
        driveTrain = swerve;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveTrain.resetOdometry(new Pose2d());
    }

    public void execute() {
        // while (driveTrain.getPoseEstimate().getX() < xTranslation) {
        //     
        //     System.out.println("Moving");
        //     System.out.println(driveTrain.getPoseEstimate().getX());
        //     driveTrain.drive(new ChassisSpeeds(1.0, 0.0, 0.0), true);
        // } 
        // driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true);
        // while (driveTrain.getPoseEstimate().getX() < xTranslation) {
        //     driveTrain.odometry.update(
        //         driveTrain.gyro.getRotation2d(), 
        //         driveTrain.getSwerveModulePositions()
        //     );
        //     driveTrain.drive(new ChassisSpeeds(1.0, 0.0, 0.0), true);
        // }
        // driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true);



        while(flag1 && !setPosition(0.25, 0.0, 0.0)) {
            driveTrain.odometry.update(
                driveTrain.gyro.getRotation2d(), 
                driveTrain.getSwerveModulePositions()
            );
        } flag1 = false;

        //cone
        //swing arm up and then outtake
        //new while block, drive backwards short distance, drop arm
        //new while block, drive backwards to pass community zone

        //cube
        //swing arm up
        //new while loop, drive forwards 16 inch, outtake
        //new while block, drive backwards short distance, drop arm
        //new while block, drive backwards to pass community zone ~110 inches




        // System.out.println("First Checkpoint");
        
        // while(flag2 && !setPosition(2.0, 0.0, 0.0)) {
        //     driveTrain.odometry.update(
        //         driveTrain.gyro.getRotation2d(), 
        //         driveTrain.getSwerveModulePositions()
        //     );
        // } flag2 = false;

        // System.out.println("Second Checkpoint");

        // while(flag3 && !setPosition(3.0, 1.0, 0.0)) {
        //     driveTrain.odometry.update(
        //         driveTrain.gyro.getRotation2d(), 
        //         driveTrain.getSwerveModulePositions()
        //     );
        // } flag3 = false;

        // System.out.println("Second Checkpoint");

        if(flag) {
            driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true);
            flag = false;
        }

    }  
    
    public boolean setPosition(double xPos, double yPos, double thetaPos) {
        double xSpeed = 0.0;
        double ySpeed = 0.0;
        double thetaSpeed = 0.0;
        boolean x = false;
        boolean y = false;
        boolean theta = false;

        if (xPos > driveTrain.getPoseEstimate().getX() + 0.03) {
            xSpeed = 0.4;
            x = false;
        } else if (xPos < driveTrain.getPoseEstimate().getX() - 0.03){
            xSpeed = -0.4;
            x = false;
        } else {
            xSpeed = 0.0;
            x = true;
        }
       
        if (yPos > driveTrain.getPoseEstimate().getY() + 0.03) {
            ySpeed = 1.0;
            y = false;
        } else if (yPos < driveTrain.getPoseEstimate().getY() - 0.03){
            ySpeed = -1.0;
            y = false;
        } else {
            ySpeed = 0.0;
            y = true;
        }

        if (thetaPos > driveTrain.getPoseEstimate().getRotation().getDegrees() + 3.0) {
            thetaSpeed = 1.0;
            theta = false;
        } else if (thetaPos < driveTrain.getPoseEstimate().getRotation().getDegrees() - 3.0){
            thetaSpeed = -1.0;
            theta = false;
        } else {
            thetaSpeed = 0.0;
            theta = true;
        }

        driveTrain.drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), true);

        return x && y && theta;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true);
    }
}
