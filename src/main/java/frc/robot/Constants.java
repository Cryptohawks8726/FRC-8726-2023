// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {
    public static final int DRIVER_XBOX = 0;
    public static final int OPERATOR_XBOX = 1;

    public static final double ARM_kP = 0.5;
    public static final double ARM_kI = 0;
    public static final double ARM_kD = 0;

    public static final int WRIST_MOTOR_SPARKMAX = 11;
    public static final int ARM_MOTOR_SPARKMAX = 12;

    public static final double RAISE_ARM_SPEED = 0.2;
    public static final double LOWER_ARM_SPEED = -0.2;

    public static final double RAISE_WRIST_SPEED = 0.5;
    public static final double LOWER_WRIST_SPEED = -0.5;

    public static final int ARM_ENCODER_CHANNEL = 3;
    public static final int WRIST_ENCODER_CHANNEL = 7;

    public static final int ENCODER_TIME_DELAY = 3; // seconds
}
