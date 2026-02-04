package org.firstinspires.ftc.teamcode.util;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware container that initializes and holds references to all robot hardware.
 * Use this class to avoid duplicating hardware initialization code across OpModes.
 */
public class RobotHardware {

    // Drive Motors
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;

    // Shooter Motors
    public DcMotorEx rightShooter;
    public DcMotorEx leftShooter;

    // Intake/Belt Motors
    public DcMotorEx intake;
    public DcMotorEx belt;

    // Servos
    public Servo ballLift;
    public Servo ballRamp;

    // CRServos (optional, for ThreeBall variant)
    public CRServo leftLift;
    public CRServo rightLift;
    public CRServo middleLift;

    // Sensors
    public IMU imu;
    public Limelight3A limelight;
    public RevBlinkinLedDriver led;

    /**
     * Initialize all standard hardware for TeleOp mode.
     * Uses DcMotorEx for velocity control.
     */
    public void initTeleOp(HardwareMap hardwareMap) {
        // Drive Motors
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_LEFT_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_LEFT_MOTOR);
        backRightDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_RIGHT_MOTOR);

        // Shooter Motors
        rightShooter = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_SHOOTER_MOTOR);
        leftShooter = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_SHOOTER_MOTOR);

        // Intake/Belt Motors
        intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        belt = hardwareMap.get(DcMotorEx.class, RobotConstants.BELT_MOTOR);

        // Servos
        ballLift = hardwareMap.get(Servo.class, RobotConstants.BALL_LIFT_SERVO);
        ballRamp = hardwareMap.get(Servo.class, RobotConstants.BALL_RAMP_SERVO);

        // LED
        led = hardwareMap.get(RevBlinkinLedDriver.class, RobotConstants.LED);

        // Set motor directions
        backRightDrive.setDirection(REVERSE);
        backLeftDrive.setDirection(REVERSE);
        rightShooter.setDirection(REVERSE);

        // Set motor modes
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);
        rightShooter.setMode(RUN_USING_ENCODER);
        leftShooter.setMode(RUN_USING_ENCODER);

        // Reset encoders
        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);
        rightShooter.setMode(STOP_AND_RESET_ENCODER);
        leftShooter.setMode(STOP_AND_RESET_ENCODER);

        // Initialize IMU
        initIMU(hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection.LEFT);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.LIMELIGHT);
    }

    /**
     * Initialize CRServos for the ThreeBall lift mechanism.
     */
    public void initCRServos(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(CRServo.class, RobotConstants.LEFT_LIFT_SERVO);
        rightLift = hardwareMap.get(CRServo.class, RobotConstants.RIGHT_LIFT_SERVO);
        middleLift = hardwareMap.get(CRServo.class, RobotConstants.MIDDLE_LIFT_SERVO);

        // Set directions
        leftLift.setDirection(REVERSE);
        middleLift.setDirection(REVERSE);
    }

    /**
     * Initialize hardware for Autonomous mode.
     * Uses DcMotor for position control, DcMotorEx for shooters.
     */
    public void initAutonomous(HardwareMap hardwareMap) {
        // Drive Motors (use DcMotor for position control)
        DcMotor flDrive = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_MOTOR);
        DcMotor frDrive = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_MOTOR);
        DcMotor blDrive = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_MOTOR);
        DcMotor brDrive = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_MOTOR);

        // Cast to DcMotorEx for consistency
        frontLeftDrive = (DcMotorEx) flDrive;
        frontRightDrive = (DcMotorEx) frDrive;
        backLeftDrive = (DcMotorEx) blDrive;
        backRightDrive = (DcMotorEx) brDrive;

        // Shooter Motors
        rightShooter = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_SHOOTER_MOTOR);
        leftShooter = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_SHOOTER_MOTOR);

        // Intake/Belt Motors (use DcMotorEx)
        intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        belt = hardwareMap.get(DcMotorEx.class, RobotConstants.BELT_MOTOR);

        // Servos
        ballLift = hardwareMap.get(Servo.class, RobotConstants.BALL_LIFT_SERVO);

        // Set motor directions - NOTE: Auto uses different reversals than TeleOp
        frontLeftDrive.setDirection(REVERSE);
        frontRightDrive.setDirection(REVERSE);
        rightShooter.setDirection(REVERSE);

        // Set motor modes
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);
        rightShooter.setMode(RUN_USING_ENCODER);
        leftShooter.setMode(RUN_USING_ENCODER);

        // Initialize IMU with different orientation for auto
        initIMU(hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT);
    }

    /**
     * Initialize the IMU with specified orientation.
     */
    private void initIMU(HardwareMap hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection logoDirection) {
        imu = hardwareMap.get(IMU.class, RobotConstants.IMU);
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Start the Limelight vision system.
     */
    public void startLimelight() {
        if (limelight != null) {
            limelight.start();
            limelight.pipelineSwitch(0);
        }
    }

    /**
     * Reset the IMU yaw to zero.
     */
    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        }
    }
}
