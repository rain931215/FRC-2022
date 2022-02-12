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
    public static final int FALCON_MAX_RPM = 6380;

    public static final int MOTOR_CHASSIS_LEFT_1 = 1;
    public static final int MOTOR_CHASSIS_LEFT_2 = 2;
    public static final int MOTOR_CHASSIS_RIGHT_1 = 3;
    public static final int MOTOR_CHASSIS_RIGHT_2 = 4;

    // These values must be tuned for your drive!
    public static final double ksVolts = 0.642;
    public static final double kvVoltSecondsPerMeter = 3.66;
    public static final double kaVoltSecondsSquaredPerMeter = 0.203;
    public static final double kPDriveVel = 1.66;
    public static final double kTrackWidthMeters = 0.609;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double ENCODER_SCALE_FACTOR_POSITION = 1.0;
    public static final double ENCODER_SCALE_FACTOR_VELOCITY = 1.0;

    public static final boolean LEFT_INVERTED = false;
    public static final boolean RIGHT_INVERTED = true;

    public static final int MOTOR_HANG = 1;
    public static final boolean MOTOR_HANG_REVERSE = false;
    public static final int MOTOR_CLIMB_1 = 1;
    public static final boolean MOTOR_CLIMB_1_REVERSE = false;
    public static final int MOTOR_CLIMB_2 = 2;
    public static final boolean MOTOR_CLIMB_2_REVERSE = false;
    // A,B channel variable for climb motors' encoder
    public static final boolean ENCODER_CLIMB_1_REVERSE = false;
    public static final boolean ENCODER_CLIMB_2_REVERSE = false;
    // Position constants for hang and climb motors
    public static final int POSITION_HANG_EXTENDED = 0;
    public static final int POSITION_CLIMB_EXTENDED = 0;


    public static final int MOTOR_SHOOT = 1;
    public static final double SHOOT_SPEED = 0.5;
    public static final int MOTOR_ROTATE = 1;
    public static final boolean MOTOR_ROTATE_INVERTED = false;
    public static final int MOTOR_ANGLE = 1;
    public static final boolean MOTOR_ANGLE_INVERTED = false;
    public static final double ALIGNMENT_X_OFFSET = 0;
    public static final double ALIGNMENT_Y_OFFSET = 0;
    public static final double[] PID_ROTATE = {0,0,0};
}
