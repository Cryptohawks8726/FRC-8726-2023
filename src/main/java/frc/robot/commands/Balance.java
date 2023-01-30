package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.SwerveDrive;



public class Balance extends CommandBase {

    public PIDController balance;
    public AHRS gyro;
    private SwerveDrive drivetrain;
    public Balance(SwerveDrive drivetrain) {
        this.drivetrain = drivetrain;
        balance = new PIDController(Constants.BalanceKp, Constants.BalanceKi, Constants.BalanceKd);
        gyro = drivetrain.getGyro();
        balance.setSetpoint(0);
        SmartDashboard.putNumber("setpoint", 0);


    }

    @Override
    public void execute() {

        double ang = gyro.getPitch();
        SmartDashboard.putNumber("robotAngle", ang);
        SmartDashboard.putNumber("balanceControlEffort", balance.calculate(ang));
        SwerveModuleState modState = new SwerveModuleState(balance.calculate(ang),Rotation2d.fromDegrees(0));
        drivetrain.setModuleStates(new SwerveModuleState[]{modState,modState,modState,modState});
        drivetrain.passiveBrake();
        //drivetrain.drive(new ChassisSpeeds(-balance.calculate(ang), 0, 0),true);

        



    }
    
}
