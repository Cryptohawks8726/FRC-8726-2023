package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.Swerve.ModulePosition.*;


public class SwerveDrive extends SubsystemBase implements Loggable, Sendable{
    
    private List<SwerveModule> modules;

    private SwerveModuleState[] modStates;
    private SwerveModulePosition[] modPositionStates;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;
    private ChassisSpeeds lastSetChassisSpeeds;
    private AHRS gyro;
    private PIDController xController,yController,thetaController;
    private SwerveModuleState testState;
    // private AnalogGyroSim simGyro;

    private Field2d field; 
    private FieldObject2d[] modPoses;
   
    public SwerveDrive(){
        modules = Arrays.asList(
            new SwerveModule(Constants.Swerve.Module.FR),
            new SwerveModule(Constants.Swerve.Module.BR),
            new SwerveModule(Constants.Swerve.Module.BL),
            new SwerveModule(Constants.Swerve.Module.FL)
        );

        modPositionStates = new SwerveModulePosition[]{
            modules.get(FR.modPos).getCurrentPosition(),
            modules.get(BR.modPos).getCurrentPosition(),
            modules.get(BL.modPos).getCurrentPosition(),
            modules.get(FL.modPos).getCurrentPosition()
        };
        
        kinematics = new SwerveDriveKinematics(
            modules.get(FR.modPos).getCenterTransform().getTranslation(),
            modules.get(BR.modPos).getCenterTransform().getTranslation(),
            modules.get(BL.modPos).getCenterTransform().getTranslation(),
            modules.get(FL.modPos).getCenterTransform().getTranslation()
        );
        
        gyro = new AHRS(SerialPort.Port.kUSB1);
        gyro.calibrate(); // possibly move to avoid the robot being moved during calibration
        gyro.reset();
        
        // simGyro = new AnalogGyroSim(0);
        
        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), modPositionStates, new Pose2d()); 
        
        lastSetChassisSpeeds = new ChassisSpeeds();
        
        field = new Field2d();
        modPoses = new FieldObject2d[]{
            field.getObject("modFR"),
            field.getObject("modBR"),
            field.getObject("modBL"),
            field.getObject("modFL")
        };
        SmartDashboard.putData("Field", field);
        logValues();

        
        /*xController = new PIDController(0.05, 0, 0,20);
        yController = new PIDController(0.05, 0, 0,20);
        thetaController = new PIDController(0.5, 0, 0,20);
        thetaController.enableContinuousInput(0, 360);*/
        testState = new SwerveModuleState(4.0,Rotation2d.fromDegrees(45));
    }

    @Override
    public void periodic(){
        odometry.update(
            gyro.getRotation2d(), 
            getSwerveModulePositions()
        );
        //modules.get(0).closedLoopDrive(testState);
        //modules.get(1).closedLoopDrive(testState);
        //modules.get(2).closedLoopDrive(testState);
        //modules.get(3).closedLoopDrive(testState);
        // modules.forEach(mod->{mod.closedLoopDrive(testState);});
        
        modules.forEach(mod->{mod.updateSteerPid();});
        logValues();
       /*  SmartDashboard.putNumber("FR ABS Encoder",modules.get(FR.modPos).getAbsPos());
        SmartDashboard.putNumber("BR ABS Encoder",modules.get(BR.modPos).getAbsPos());
        SmartDashboard.putNumber("BL ABS Encoder",modules.get(BL.modPos).getAbsPos());
        SmartDashboard.putNumber("FL ABS Encoder",modules.get(FL.modPos).getAbsPos());*/
        
        // show estimated robot and mod poses on dashboard
        field.setRobotPose(odometry.getEstimatedPosition());
        /*for (int i = 0;i<4;i++){
            modPoses[i].setPose(
                odometry.getEstimatedPosition()
                .plus(
                    modules.get(i).getCenterTransform()
                    .plus
                        (new Transform2d(new Translation2d(),modules.get(i).getCurrentState().angle))
                    )
            );
        }*/
        
       // logValues();
       // System.out.println("---------------");
       // System.out.print("Yaw: ");
       // System.out.println(gyro.getYaw());
        //System.out.println(gyro.getRotation2d().getDegrees());
    }
    /* 
    @Override
    public void simulationPeriodic(){
        
        // multiplying by 0.001 makes it more usable, it isn't based on an accurate time interval
        simGyro.setAngle(simGyro.getAngle() + lastSetChassisSpeeds.omegaRadiansPerSecond*57.2958*0.001);
        odometry.update(
            new Rotation2d(simGyro.getAngle()), 
            getSwerveModulePositions()
        );

        Pose2d estimatedPostition = odometry.getEstimatedPosition();

        field.setRobotPose(estimatedPostition);
        // sim new positions of modules
        for (int i = 0;i<4;i++){
            modPoses[i].setPose(
                estimatedPostition // current robot origin
                .plus(
                    modules.get(i).getCenterTransform() // transform by constant translation to module
                    .plus
                        (new Transform2d(new Translation2d(),modules.get(i).getLastSetState().angle)) // set mod rotation to the last set angle
                    )
            );
        }
        SmartDashboard.putNumber("xpos", estimatedPostition.getTranslation().getX());
        SmartDashboard.putNumber("ypos", estimatedPostition.getTranslation().getY());
        SmartDashboard.putNumber("setXVel", lastSetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("setYVel", lastSetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("FRvel", modules.get(FR.modPos).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("BRvel", modules.get(BR.modPos).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("BLvel", modules.get(BL.modPos).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("FLvel", modules.get(FL.modPos).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("FRdeg", modules.get(FR.modPos).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("BRdeg", modules.get(BR.modPos).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("BLdeg", modules.get(BL.modPos).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("FLdeg", modules.get(FL.modPos).getLastSetState().angle.getDegrees());
    };*/

    public void drive(ChassisSpeeds robotSpeeds, boolean isClosedLoop){  
        lastSetChassisSpeeds = robotSpeeds;
        modStates = kinematics.toSwerveModuleStates(robotSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(modStates,Constants.Swerve.maxSpeed);
       /* if (robotSpeeds.vxMetersPerSecond == 0.0 && robotSpeeds.vyMetersPerSecond == 0.0 && robotSpeeds.omegaRadiansPerSecond == 0.0){
            
            for (int i = 0; i<4;i++){
                modStates[i] = new SwerveModuleState(modStates[i].speedMetersPerSecond,Rotation2d.fromDegrees(0));
            }
        }*/

       /*if (robotSpeeds.vxMetersPerSecond == 0.0 && robotSpeeds.vyMetersPerSecond == 0.0){
            
            
                modStates[1] = new SwerveModuleState(-modStates[1].speedMetersPerSecond,modStates[1].angle);
                modStates[3] = new SwerveModuleState(-modStates[3].speedMetersPerSecond,modStates[3].angle);

        
        }*/
        modules.forEach(mod -> {mod.closedLoopDrive(modStates[mod.getModPos().getVal()]);});
    }

    public StartEndCommand passiveBrake(){
        SwerveModuleState leftToRight = new SwerveModuleState(0.0,Rotation2d.fromDegrees(45));
        SwerveModuleState rightToLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
        return new StartEndCommand(
            () -> modules.forEach(mod -> {mod.closedLoopDrive((mod.getModPos().getVal() %2 == 0) ? leftToRight : rightToLeft).setBrake();}), 
            () -> modules.forEach(mod -> {mod.setCoast();}), 
            this
        );
    }

    public AHRS getGyro(){
        return gyro;
    }

    public void normalZeroModules(){
        modules.forEach(mod -> {mod.closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(0)));});
    }

    public Pose2d getPoseEstimate(){
        return odometry.getEstimatedPosition();
    }

    public void setOdometryPosition(Pose2d setPosition){
        odometry.resetPosition(getRobotAngle(), getSwerveModulePositions(), setPosition);
    }

    public Rotation2d getRobotAngle() {
        // return Rotation2d.fromDegrees(gyro.getYaw());
        return gyro.getRotation2d();
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        modules.forEach(mod -> {modPositionStates[mod.getModPos().getVal()] = mod.getCurrentPosition();});
        return modPositionStates;
    }

    public void setModuleStates(SwerveModuleState[] updatedstates){
        SwerveDriveKinematics.desaturateWheelSpeeds(updatedstates, Constants.Swerve.maxSpeed);
        modules.forEach(mod -> {mod.closedLoopDrive(updatedstates[mod.getModPos().getVal()]);});
    }

    public void setEncoderOffsets(){
        modules.forEach(mod -> {mod.setEncoderOffset();});
    }

    public void logValues(){ 
        Pose2d estimatedPostition = odometry.getEstimatedPosition();

        SmartDashboard.putNumber("xpos", estimatedPostition.getTranslation().getX());
        SmartDashboard.putNumber("ypos", estimatedPostition.getTranslation().getY());
        SmartDashboard.putNumber("estimatedthetaPos",estimatedPostition.getRotation().getDegrees());
        SmartDashboard.putNumber("gyroAngle", gyro.getYaw());//getRotation2d().getDegrees()%360
        SmartDashboard.putBoolean("isGyroConnected", gyro.isConnected());
        SmartDashboard.putNumber("setXVel", lastSetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("setYVel", lastSetChassisSpeeds.vyMetersPerSecond);
        
        for (SwerveModule module : modules) {
            String modName = module.getModPos().toString();
            module.seedRelativeEncoder();
            SmartDashboard.putNumber(modName + "setvel", module.getLastSetState().speedMetersPerSecond);
            SmartDashboard.putNumber(modName + "actvel", module.getCurrentState().speedMetersPerSecond);
            SmartDashboard.putNumber(modName + "setdeg", module.gettSetStateAngle());
            SmartDashboard.putNumber(modName + "actdeg", module.getCurrentState().angle.getDegrees());
            SmartDashboard.putNumber(modName + "absdeg", module.getAbsPos());
            SmartDashboard.putNumber(modName + "built in steer", module.getRelativePos());
            SmartDashboard.putNumber(modName + "built in drive", module.getRelativeVel());
            SmartDashboard.putNumber(modName + "drive current", module.getDriveCurrent());
            SmartDashboard.putNumber(modName + "steer current", module.getSteerCurrent());
            
        }

    }

   // public SequentialCommandGroup driveToPos(){
    //    return new InstantCommand(()->{xController.reset();})
   // }

    @Log
    public double XCoordinate(){
        return odometry.getEstimatedPosition().getX();
    }

    @Log
    public double YCoordinate(){
        return odometry.getEstimatedPosition().getY();
    }

    @Log
    public double Module0Speed(){
       return modules.get(0).getCurrentState().speedMetersPerSecond;
    }

    @Log
    public double Module1Speed(){
       return modules.get(1).getCurrentState().speedMetersPerSecond;
    }

    @Log
    public double Module2Speed(){
       return modules.get(2).getCurrentState().speedMetersPerSecond;
    }

    @Log
    public double Module3Speed(){
       return modules.get(3).getCurrentState().speedMetersPerSecond;
    }

    @Log
    public double Module0Angle(){
        return modules.get(0).getCurrentState().angle.getDegrees();
    }

    @Log
    public double Module1Angle(){
        return modules.get(1).getCurrentState().angle.getDegrees();
    }

    @Log
    public double Module2Angle(){
        return modules.get(2).getCurrentState().angle.getDegrees();
    }

    @Log
    public double Module3Angle(){
        return modules.get(3).getCurrentState().angle.getDegrees();
    }    
}
