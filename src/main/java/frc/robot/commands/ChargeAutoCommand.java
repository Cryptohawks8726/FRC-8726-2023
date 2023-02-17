package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;

import java.util.TimerTask;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class ChargeAutoCommand extends CommandBase{
    
    private AHRS gyro;
    
    private SwerveDrive drivetrain;
    public Timer timer;

    public ChargeAutoCommand(SwerveDrive drivetrain){
        
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        timer = new Timer();
        gyro = ahrsGyro;
    }

    @Override
    public void initialize(){
        gyro.calibrate(); 
        gyro.reset();
        timer.start();
    }

    @Override
    public void execute(){
        double xVel = 5.00;
        double yVel = 5.00;
        drivetrain.drive(new ChassisSpeeds(xVel, yVel, 0.00), false);
    }

    @Override
    public boolean isFinished(){

        if(timer.get() > 4.00){
            return true;
        }

        return false;
    }

}

