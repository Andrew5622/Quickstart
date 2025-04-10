package org.firstinspires.ftc.teamcode.utilities.robot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

@Config
public class DriveConstants {

    public enum ParkPositions {
        LEFT,
        RIGHT
    }

    public static final double DEAD_WHEEL_SIZE = 1.375;

    public static final double TICKS_PER_REVOLUTION = 2000;

    public static final double WHEEL_TICKS = 2000;
    public static final double WHEEL_SIZE = 1.89/2;

    public static final double INCHES_PER_REVOLUTION = 2 * WHEEL_SIZE * Math.PI;
    public static final double TICKS_PER_INCH = 1 / (2 * WHEEL_SIZE * Math.PI) * DriveConstants.INCHES_PER_REVOLUTION;
    public static final double CONVERSION_CONSTANT = (1 / DriveConstants.TICKS_PER_REVOLUTION) * DriveConstants.WHEEL_SIZE * 2 * Math.PI;


    public static double getEncoderTicksFromInches(double inches) {
        return (inches / DriveConstants.INCHES_PER_REVOLUTION) * DriveConstants.WHEEL_TICKS;
    }

    public static double getInchesFromEncoderTicks(double ticks) {
        return (ticks / DriveConstants.WHEEL_TICKS) * DriveConstants.INCHES_PER_REVOLUTION;
    }

    public static double BANG_BANG_POWER = -0.5;
    public static double TICK_THRESHOLD = 50;
    public static double ANGLE_AT_TIME = 0;
    public static double MAX_TURN_TIME = 1.75;

    public static double TURN_THRESHOLD = Math.toRadians(2);
    public static double ANGULAR_VELOCITY_THRESHOLD = Math.toRadians(10);
    public static double ANGULAR_VELOCITY_THRESHOLD_MIN = Math.toRadians(1);

    public static double MAX_CORRECTION_TIME = 0.5;

    public static double V_MAX = 50; // If doing 1
    public static double A_MAX = 40;

    public static double K_V = 0.013;
    public static double K_A = 0.0035;

    public static Pose THRESHOLD = new Pose(1, 1, Math.toRadians(1));
    public static double THRESHOLD_TIME = 0.5;

    public static double MAX_ANGULAR_VELOCITY = Math.toRadians(180);
    public static double trackWidth = 14;

    public static ParkPositions parkPosition = ParkPositions.RIGHT;

}
