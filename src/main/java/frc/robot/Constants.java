// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final int DRIVER_XBOX = 0;
    public static final int OPERATOR_XBOX = 1;

    public static final double ARM_kP = 2;
    public static final double ARM_kI = 0;
    public static final double ARM_kD = 0;

    //Change values after testing
    public static final double CONEHELD_kP = 0;
    public static final double CONEHELD_kI = 0;
    public static final double CONEHELD_kD = 0;

    public static final int ARM_MOTOR_SPARKMAX = 12;

    public static final double RAISE_ARM_SPEED = 0.2;
    public static final double LOWER_ARM_SPEED = -0.2;

    public static final double ARM_TEST_PRESET = 0.6;

    //DIO Input for the Arm Encoder
    public static final int ENCODER_GREEN = 0;
    public static final int ENCODER_YELLOW = 1;
    public static final int ENCODER_BLUE = 2;
}
