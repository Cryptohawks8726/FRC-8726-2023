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

    public static final double ARM_kP = 0.005;
    public static final double ARM_kI = 0;
    public static final double ARM_kD = 0;

    public static final int ARM_MOTOR_SPARKMAX = 51;

    public static final double RAISE_ARM_SPEED = 0.5;
    public static final double LOWER_ARM_SPEED = -0.5;

    private static NetworkTableInstance instance;
    private static NetworkTable table;
    public static NetworkTableEntry armCurr;
    public static void setUpNT(){
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("debugValues");
        armCurr = table.getEntry("armCurr");
      }
}
