// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //PID Constants
    public static final double INTAKE_kP = 0.2;
    public static final double INTAKE_kI = 0.0;
    public static final double INTAKE_kD = 0.0;

    private static NetworkTableInstance instance;
    private static NetworkTable table;
    // (unused) private static NetworkTableEntry intakeSpeed, outtakeSpeed,intakePeak,intakeCont,armPeak,armCont;
    public static NetworkTableEntry intakeCurr1,intakeCurr2;
    public static void setUpNT(){
      instance = NetworkTableInstance.getDefault();
      table = instance.getTable("debugValues");
      intakeCurr1 = table.getEntry("intakeCurr1");
      intakeCurr2 = table.getEntry("intakeCurr2");

    }
}
