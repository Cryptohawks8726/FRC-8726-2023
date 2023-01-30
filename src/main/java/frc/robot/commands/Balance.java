package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;


public class Balance extends CommandBase {

    public PIDController balance;
    public AHRS gyro;

    public Balance() {
        balance = new PIDController(Constants.BalanceKp, Constants.BalanceKi, Constants.BalanceKd);
        gyro = new AHRS(SPI.Port.kMXP);
        balance.setSetpoint(0);


    }

    @Override
    public void execute() {

        double ang = gyro.getAngle();
        



    }
    
}
