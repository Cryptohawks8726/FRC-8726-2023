// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Swerve {
        
        // Physical Constants
        public static final double driveBaseWidth = 0.762;
        public static final double driveBaseLength = 0.762; //meters
        public static final double driveGearRatio = 6.12; // L3
        public static final double steerGearRatio = 12.8; 
        public static final double wheelDiameterMeters = 0.098; // Measure and check later. Compensating for tread wear over comp could be cool
        public static final double driveConversionFactor = wheelDiameterMeters * Math.PI / driveGearRatio;

        public static final double maxSpeed = 3.0; // m/s, I have no clue if this is realistic // TODO testing
        public static final double maxAngularSpeed = 2; // rad/s
        public static final double driverThetaDeadband = 0.05;
        public static final double driverTranslationDeadband = 1;
        // Electrical Constants
        public static final int  driveMotorFreeCurrentLimit = 30;
        public static final int  driveMotorStallCurrentLimit = 30;
        public static final int  driveMotorfreeCurrentLimit = 30;
        
        // Controller Gains
        // TODO: Tune PID + FF constants
        public static final double kDriveP = 0.35;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 0.0;

        public static final double kSteerP = 0.005;//0.0025;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.0;
        public static final double kSteerFF = 0.0;

        public static final double kHeadingP = 0.01;
        public static final double kHeadingI = 0.0;
        public static final double kHeadingD = 0.0;
        public static final double kHeadingFF = 0.0;

         //system identification constants
        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        public static final double kPDriveVel = 0;
        public static final double kTrackwidthMeters = 0;

        //"trackwidthmeters" =  public static final double driveBaseWidth = 0.762;
        public static final double kMaxSpeedMetersPerSecond = 25.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;

        //autoconstants
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(driveBaseWidth/2, -driveBaseLength/2), 
            new Translation2d(driveBaseWidth/2, driveBaseLength/2),
            new Translation2d(-driveBaseWidth/2, -driveBaseLength/2),
            new Translation2d(-driveBaseWidth/2, driveBaseLength/2)
        );
        
        public static final double kPXController = 1;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        (2*2*Math.PI)/10, //maximumangularspeed
                        3 //maxangularaccelerationradiansperseconds);
                );

        
        // Module Constants
        public enum ModulePosition{
            FR(0),BR(1),BL(2),FL(3);
            public final int modPos;
            
            private ModulePosition(int modPos){
                this.modPos = modPos;
            }

            public int getVal(){
                return this.modPos;
            }

            

        }
        public enum Module{
            /*       
            From Translation2d docs: 
            When the robot is placed on the origin, facing toward the X direction, 
            moving forward increases the X, whereas moving to the left increases the Y.

            Mod{modPos,driveMotorid,steerMotorid,cancoderid,displacment(x,y)}

            126.826
            43.418
            -47.549
            228.340
            */

            //TODO: Define Forward
            FR(ModulePosition.FR,10,11,12,-92.109375,new Transform2d(new Translation2d(driveBaseLength/2,-driveBaseWidth/2),new Rotation2d())), 
            BR(ModulePosition.BR,20,21,22,178.197266,new Transform2d(new Translation2d(-driveBaseLength/2,-driveBaseWidth/2),new Rotation2d())),
            BL(ModulePosition.BL,30,31,32,-7.734375,new Transform2d(new Translation2d(-driveBaseLength/2,driveBaseWidth/2),new Rotation2d())),
            FL(ModulePosition.FL,40,41,42,23.554688,new Transform2d(new Translation2d(driveBaseLength/2,driveBaseWidth/2),new Rotation2d())); 
            
            public final ModulePosition modPos;
            public final int driveMotorid;
            public final int steerMotorid;
            public final int canCoderid;
            public final double canCoderOffset;
            public final Transform2d displacment; // from robot origin
            
            private Module(ModulePosition modPos, int driveMotorid, int steerMotorid, int canCoderid,double canCoderOffset, Transform2d displacment){
                this.modPos = modPos;
                this.driveMotorid = driveMotorid;
                this.steerMotorid = steerMotorid;
                this.canCoderid = canCoderid;
                this.canCoderOffset = canCoderOffset;
                this.displacment = displacment;
            }

        }
      
    }
    public final class PhotonVisionConstants{
        // Constants such as camera and target height stored. Change per robot and goal!
        // Actual values will be dependent on final bot
        // Measured in meters
        public static final double CAMERA_HEIGHT_METERS = .5;
        public static final double TARGET_HEIGHT_METERS = 1;
        // Angle between horizontal and the camera.
        public static final double CAMERA_PITCH_RADIANS = 1;
        // How far from the target we want to be
        public static final double GOAL_RANGE_METERS = 1;

        public static final double kCameraHeight = 1;
        public static final double kTargetHeight = 1;
        public static final double kCameraPitch = 0;
        public static final double kTargetPitch = 0;

        public static final double default2dPose_X = 0;
        public static final double default2dPose_Y = 0;
        //public static final Rotation2d default2dPose_rotation = new Rotation2d(0);



    }
}
