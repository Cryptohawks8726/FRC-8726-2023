package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.SwerveDrive;



public class Balance extends CommandBase {

    double angTolerance = 2;
    public PIDController balance;
    public AHRS gyro;
    private boolean isFinished;
    private SwerveDrive drivetrain;
    private Command passiveBrake;
    public Balance(SwerveDrive drivetrain) {
        this.drivetrain = drivetrain;
        balance = new PIDController(Constants.BalanceKp, Constants.BalanceKi, Constants.BalanceKd);
        gyro = drivetrain.getGyro();
        passiveBrake = drivetrain.passiveBrake();
        balance.setSetpoint(0);
        SmartDashboard.putNumber("setpoint", 0);
        isFinished = false;


    }

    @Override
    public void execute(){
        double ang = gyro.getPitch();
        SmartDashboard.putNumber("robotAngle", ang);
        SmartDashboard.putNumber("balanceControlEffort", balance.calculate(ang));
        if (ang >= -angTolerance && ang <= angTolerance) {
            SwerveModuleState modState = new SwerveModuleState(0,Rotation2d.fromDegrees(0));
            drivetrain.setModuleStates(new SwerveModuleState[]{modState,modState,modState,modState});
            System.out.println("stable");
            passiveBrake.schedule();
            //drivetrain.passiveBrake();
            isFinished = true;
        } else {
            isFinished=false;
            SwerveModuleState modState = new SwerveModuleState(balance.calculate(ang),Rotation2d.fromDegrees(0));
            drivetrain.setModuleStates(new SwerveModuleState[]{modState,modState,modState,modState});
        }
    }


    @Override
    public void end(boolean isInterrupted){
        //passiveBrake.end(isInterrupted);
    }
    /*public void balancer() {

        double ang = gyro.getPitch();
        SmartDashboard.putNumber("robotAngle", ang);
        SmartDashboard.putNumber("balanceControlEffort", balance.calculate(ang));
        if (ang >= -angTolerance && ang <= angTolerance) {
            SwerveModuleState modState = new SwerveModuleState(0,Rotation2d.fromDegrees(0));
            drivetrain.setModuleStates(new SwerveModuleState[]{modState,modState,modState,modState});
            drivetrain.passiveBrake();
        } else {
            SwerveModuleState modState = new SwerveModuleState(balance.calculate(ang),Rotation2d.fromDegrees(0));
            drivetrain.setModuleStates(new SwerveModuleState[]{modState,modState,modState,modState});
        }
        //drivetrain.drive(new ChassisSpeeds(-balance.calculate(ang), 0, 0),true);
    }*/
}
