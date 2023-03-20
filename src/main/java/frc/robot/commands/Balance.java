package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.SwerveDrive;

public class Balance extends CommandBase {

    double angTolerance = 2;
    public PIDController balance;
    public AHRS gyro;
    private Boolean isBalanced;
    private SwerveDrive drivetrain;
    private Command passiveBrake;
    public Balance(SwerveDrive drivetrain) {
        this.drivetrain = drivetrain;
        balance = new PIDController(Constants.BalanceKp, Constants.BalanceKi, Constants.BalanceKd);
        gyro = drivetrain.getGyro();
        passiveBrake = drivetrain.passiveBrake();
        balance.setSetpoint(0);
        isBalanced = false;
    }

    @Override
    public void execute(){
        double ang = gyro.getPitch();
        SmartDashboard.putNumber("Balance Angle", ang);
        SmartDashboard.putNumber("balanceControlEffort", balance.calculate(ang));
        if (Math.abs(ang)<=angTolerance) {
            //SwerveModuleState modState = new SwerveModuleState(0,Rotation2d.fromDegrees(0));
            //drivetrain.setModuleStates(new SwerveModuleState[]{modState,modState,modState,modState});
            System.out.println("stable");
            isBalanced = true;
            //if(!passiveBrake.isScheduled()){
           //     passiveBrake.schedule();
           // }
        } else {
            double speed = 0.0;
            if(Math.abs(balance.calculate(ang))>0.4){
                speed = Math.signum(balance.calculate(ang))*0.4;
            }
            SwerveModuleState modState = new SwerveModuleState(speed,Rotation2d.fromDegrees(0));
            drivetrain.setModuleStates(new SwerveModuleState[]{modState,modState,modState,modState});
        }
    }
     @Override 
     public boolean isFinished(){
        return isBalanced;
     }

    @Override
    public void end(boolean isInterrupted){
        passiveBrake.cancel();
    }
}
