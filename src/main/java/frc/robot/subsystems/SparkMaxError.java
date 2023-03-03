package frc.robot.subsystems;

import java.util.List;
import java.util.Arrays;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class SparkMaxError extends SubsystemBase{

    private List<CANSparkMax> motorControllers;
    //private CANSparkMax swerveSparkFRD, swerveSparkFRS, swerveSparkBRD, swerveSparkBRS, swerveSparkFLD, swerveSparkFLS, swerveSparkBLD, swerveSparkBLS; 
    //private CANSparkMax armIntake, armSubsystem, groundIntakeLeft, groundIntakeRight;

    public SparkMaxError(List<SwerveModule> swerveDriveModules, CANSparkMax armIntake, CANSparkMax armSubsystem, CANSparkMax groundIntakeLeft, CANSparkMax groundIntakeRight){
        
        motorControllers = Arrays.asList(
            swerveDriveModules.get(0).getSparkMaxDriverMotor(),
            swerveDriveModules.get(0).getSparkMaxSteerMotor(),
            swerveDriveModules.get(1).getSparkMaxDriverMotor(),
            swerveDriveModules.get(1).getSparkMaxSteerMotor(),
            swerveDriveModules.get(2).getSparkMaxDriverMotor(),
            swerveDriveModules.get(2).getSparkMaxSteerMotor(),
            swerveDriveModules.get(3).getSparkMaxDriverMotor(),
            swerveDriveModules.get(3).getSparkMaxSteerMotor(),
            armIntake, armSubsystem, groundIntakeLeft, groundIntakeRight);

    }

    @Override
    public void periodic(){
        getError();
    }

    public void getError(){

        List<String> sparkList = Arrays.asList("swerveSparkFRD", "swerveSparkFRS", 
        "swerveSparkBRD", "swerveSparkBRS", "swerveSparkFLD","swerveSparkFLS", 
        "swerveSparkBLD", "swerveSparkBLS", "armIntake", "armSubsystem", "groundIntakeLeft", "groundIntakeRight");

        int i = 0; 
        for (CANSparkMax controller : motorControllers){
        
            if (controller.getFault(CANSparkMax.FaultID.kSensorFault)){
                SmartDashboard.putString(sparkList.get(i), "kSensorFault");
            }
            else if(controller.getFault(CANSparkMax.FaultID.kMotorFault)){
                SmartDashboard.putString(sparkList.get(i), "kMotorFault");
            }
            i++;
        }

        return; 
    }
}
 