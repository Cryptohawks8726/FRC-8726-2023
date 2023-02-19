// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int GROUND_INTAKE_LEFT_MOTOR = 0;
    public static final int GROUND_INTAKE_RIGHT_MOTOR = 0;

    public static final int GROUND_INTAKE_LOWER_PISTON = 0;
    public static final int GROUND_INTAKE_UPPER_PISTON = 0;

    public static final double GROUND_INTAKE_WHEEL_SPEED = 0.3;

    public static final int WRIST_MOTOR_SPARKMAX = 0;

    public static final double WRIST_kP = 0.05;
    public static final double WRIST_kI = 0.0;
    public static final double WRIST_kD = 0.0;

    public static final double RAISE_WRIST_SPEED = 0.5;
    public static final double LOWER_WRIST_SPEED = -0.5;

    public static final double WRIST_RETRACT_POS = 0.0;
    public static final double WRIST_EXTEND_POS = 0.0;

    public static final int COMPRESSOR_ID = 0;

    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERTOR_CONTROLLER = 0;

    public static final double WRIST_ENCODER_OFFSET = 0.0;

    public static final int[] YELLOW_RGB = new int[] {255, 255, 0};
    public static final int[] PURPLE_RGB = new int[] {75, 0, 130};

    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 0;
}
