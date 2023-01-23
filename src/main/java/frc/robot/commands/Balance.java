package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

public class Balance extends CommandBase {

    

    public Balance() {
        PIDController balance = new PIDController(Constants.BalanceKp, Constants.BalanceKi, Constants.BalanceKd);
        AHRS gyro = new AHRS(SPI.Port.kMXP);


    }
    
}
