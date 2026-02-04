package org.firstinspires.ftc.teamcode.util;

/**
 * Centralized constants for the robot hardware and configuration.
 * All hardware names, encoder values, and servo positions are defined here.
 */
public final class RobotConstants {

    private RobotConstants() {} // Prevent instantiation

    // ==================== Hardware Names ====================

    // Drive Motors
    public static final String FRONT_LEFT_MOTOR = "FrontLeft";
    public static final String FRONT_RIGHT_MOTOR = "FrontRight";
    public static final String BACK_LEFT_MOTOR = "BackLeft";
    public static final String BACK_RIGHT_MOTOR = "BackRight";

    // Shooter Motors
    public static final String RIGHT_SHOOTER_MOTOR = "RightShooter";
    public static final String LEFT_SHOOTER_MOTOR = "LeftShooter";

    // Intake/Belt Motors
    public static final String INTAKE_MOTOR = "Intake";
    public static final String BELT_MOTOR = "Belt";

    // Servos
    public static final String BALL_LIFT_SERVO = "BallLift";
    public static final String BALL_RAMP_SERVO = "BallRamp";

    // CRServos (for ThreeBall lift mechanism)
    public static final String LEFT_LIFT_SERVO = "LeftLift";
    public static final String RIGHT_LIFT_SERVO = "RightLift";
    public static final String MIDDLE_LIFT_SERVO = "MiddleLift";

    // Sensors
    public static final String IMU = "imu";
    public static final String LIMELIGHT = "limelight";
    public static final String LED = "LED";

    // ==================== Encoder Constants ====================

    // Close Auto Mode
    public static final double CLOSE_COUNTS_PER_INCH = 29.8;
    public static final double CLOSE_COUNTS_PER_DEGREE = 7.4;

    // Far Auto Mode
    public static final double FAR_COUNTS_PER_INCH = 41.0;
    public static final double FAR_COUNTS_PER_DEGREE = 10.5;

    // ==================== Servo Positions ====================

    // Ball Lift Servo
    public static final double BALL_LIFT_REST = 0.06;
    public static final double BALL_LIFT_UP = 0.18;  // 0.12 + 0.06
    public static final double BALL_LIFT_AUTO_UP = 0.3;

    // Ball Ramp Servo
    public static final double BALL_RAMP_REST = 0.07;
    public static final double BALL_RAMP_UP = 0.13;  // 0.07 + 0.06

    // ==================== Motor Speeds ====================

    public static final int MAX_DRIVE_VELOCITY = 2000;
    public static final int MAX_SHOOTER_VELOCITY = 2000;
    public static final int DEFAULT_SHOOTER_VELOCITY = 1000;
    public static final int VELOCITY_STEP = 20;

    // ==================== Vision Constants ====================

    // Limelight configuration
    public static final double CAMERA_HEIGHT_IN = 10.0;
    public static final double TAG_HEIGHT_IN = 29.5;
    public static final double CAMERA_PITCH_DEG = 22.5;

    // Vision tolerances
    public static final double TOWER_ANGLE_TOLERANCE = 4.0;
    public static final double VISION_STALENESS_THRESHOLD = 50.0;

    // AprilTag IDs
    public static final int BLUE_TOWER_TAG_ID = 20;
    public static final int RED_TOWER_TAG_ID = 24;
    public static final int BALL_ORDER_GPP_TAG_ID = 21;
    public static final int BALL_ORDER_PGP_TAG_ID = 22;
    public static final int BALL_ORDER_PPG_TAG_ID = 23;

    // Velocity calculation coefficients
    public static final double VELOCITY_SLOPE = 0.260102;
    public static final double VELOCITY_INTERCEPT = 34.80803;
}
