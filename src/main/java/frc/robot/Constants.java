// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;


public final class Constants {
    public static final int OPERATOR_XBOX = 1;
    public static final int DRIVER_CONTROLLER = 0;

    //Balance PID Constants
    public static final double BalanceKp= 0.6;
    public static final double BalanceKi= 0.00;
    public static final double BalanceKd= 0.00;

    
    
    public static final class Arm{
        
        public static final int ARM_SPARKMAX = 56;
        public static final int BRAKE_PISTON = 9;
        
        public static final double SHAFT_HEIGHT_INCHES = 40.625;
        public static final double ARM_LENGTH_INCHES = 35.0;
        
        // PID and FF for voltage

        public static final double ARM_kVP = 4.8;//maybe increase after kv
        public static final double ARM_kVI = 0.0;
        public static final double ARM_kVD = 0.0;
        public static final double ARM_kS = 0.545;//545;
        public static final double ARM_kV = 0.95;//increase kv
        public static final double ARM_kG = 1.2;//

        public static final double SPEED_RAD = 1.0; // increase later

        //deg
        public static final double HIGHNODE_ANGLE = 10.0+2;
        public static final double MID_ANGLE = -6.0;
        public static final double MID_CUBE_ANGLE = -9.0; // remove?
        public static final double SHELF_CONE = 3.0; 
        public static final double SHELF_CUBE = 1.0;
        public static final double RETRACTED_ANGLE = -95.0;
        public static final double FLOOR_ANGLE = -85.0;
        public static final double SUBSTATION_CUBE_ANGLE = -57.0;
        public static final double SUBSTATION_CONE_ANGLE = -31.5;
        

        public static final double ENCODER_OFFSET_CONFIG = 335.0433326;
        public static final double ENCODER_OFFSET_SUBTRACT = 311.525;
        public static final double ENCODER_POS_FACTOR = 360;
    }

    
    public static final class GroundIntake{
        public static final int LEFT_SPARKMAX = 57;
        public static final int RIGHT_SPARKMAX = 58;

        public static final int CLAMP_PISTON = 12;
        public static final int UPPER_PISTON = 7;
        public static final int LOWER_PISTON = 8;

        public static final double WHEEL_SPEED = 0.2;
    }

    public static final class ArmIntake{
        public static final int WRIST_SPARKMAX = 55;
        public static final int ARM_INTAKE_SPARKMAX = 50;

        public static final double WRIST_kP = 6.0;
        public static final double WRIST_kI = 0.0;
        public static final double WRIST_kD = 0.0;

        public static final double RAISE_WRIST_SPEED = 0.5;
        public static final double LOWER_WRIST_SPEED = -0.5;

        public static final double WRIST_RETRACT_POS = 0.245;
        public static final double EXTEND_DIFF = 0.160;// need to tune
        public static final double SHELF_DIFF = 0.150; //need to tune
        
        public static final double WRIST_ENCODER_OFFSET = 0.6028818;
    }

    public static final class Swerve {
        
        // Physical Constants
        public static final double driveBaseWidth = 0.762;
        public static final double driveBaseLength = 0.762; //meters
        public static final double driveGearRatio = 6.12; // L3
        public static final double steerGearRatio = 12.8; 
        public static final double wheelDiameterMeters = 0.098; // Measure and check later. Compensating for tread wear over comp could be cool
        public static final double driveConversionFactor = wheelDiameterMeters * Math.PI / driveGearRatio;

        public static final double maxSpeed = 5.0; 
        public static final double maxAngularSpeed = 2.0; 
        public static final double driverThetaDeadband = 0.05;
        public static final double driverTranslationDeadband = 1;
        // Electrical Constants
        public static final int  driveMotorFreeCurrentLimit = 30;
        public static final int  driveMotorStallCurrentLimit = 30;
        public static final int  driveMotorfreeCurrentLimit = 30;

        // Controller Gains
        // TODO: Tune PID + FF constants
        public static final double kDriveP = 0.55;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 0.0;

        public static final double kSteerP = 0.005;//0.0025;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.0;
        public static final double kSteerFF = 0.0;

        public static final double kHeadingP = 0.5;
        public static final double kHeadingI = 0.0;
        public static final double kHeadingD = 0.0;
        public static final double kHeadingFF = 0.0;
        
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
            FR(ModulePosition.FR,10,11,12,-256.5/*-92.109375*/,new Transform2d(new Translation2d(driveBaseLength/2,-driveBaseWidth/2),new Rotation2d())), 
            BR(ModulePosition.BR,20,21,22,-29.61914/*182.197266*/,new Transform2d(new Translation2d(-driveBaseLength/2,-driveBaseWidth/2),new Rotation2d())),
            BL(ModulePosition.BL,30,31,32,-111.5332/*-187.734375*/,new Transform2d(new Translation2d(-driveBaseLength/2,driveBaseWidth/2),new Rotation2d())),
            FL(ModulePosition.FL,41,40,42,-238.008/*23.554688*/,new Transform2d(new Translation2d(driveBaseLength/2,driveBaseWidth/2),new Rotation2d())); 
            
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

    public static final int COMPRESSOR_ID = 1;

    public static final int[] YELLOW_RGB = new int[] {255, 255, 0};
    public static final int[] PURPLE_RGB = new int[] {75, 0, 130};
    public static final int[] WHITE_RGB = new int[] {255,255,255};
    public static final int[] OFF_RGB = new int[] {0,0,0};
    public static final int LED_PORT = 9;
    public static final int LED_LENGTH = 37;
}
