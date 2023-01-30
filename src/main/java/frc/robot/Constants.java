// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Constants {
    public static final int DRIVER_XBOX = 0;
    public static final int OPERATOR_XBOX = 1;

    public static final double ARM_kP = 0.01;
    public static final double ARM_kI = 0;
    public static final double ARM_kD = 0;

    public static final double HAND_kP = 0.0005;
    public static final double HAND_kI = 0;
    public static final double HAND_kD = 0;

    public static final int ARM_MOTOR_SPARKMAX = 12;
    public static final int HAND_MOTOR_SPARKMAX = 13;

    public static final double RAISE_ARM_SPEED = 0.2;
    public static final double LOWER_ARM_SPEED = -0.2;

    public static final double RAISE_HAND_SPEED = 0.3;
    public static final double LOWER_HAND_SPEED = -0.3;

    //DIO Input for the Arm Encoder
    public static final int ENCODER_GREEN = 0;
    public static final int ENCODER_YELLOW = 1;
    public static final int ENCODER_BLUE = 2;
  
    // private static NetworkTableInstance instance;
    // private static NetworkTable table;
    // public static NetworkTableEntry armCurr;
    // public static NetworkTableEntry handCurr;
    // public static void setUpNT(){
    //   instance = NetworkTableInstance.getDefault();
    //   table = instance.getTable("debugValues");
    //   armCurr = table.getEntry("armCurr");
    //   handCurr = table.getEntry("handCurr");
    // }
}
