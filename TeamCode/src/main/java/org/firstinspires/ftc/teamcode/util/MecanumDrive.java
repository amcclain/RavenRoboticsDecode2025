package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drive controller for TeleOp mode.
 * Provides velocity-based driving with field-relative and robot-relative options.
 */
public class MecanumDrive {

    private final DcMotorEx frontLeftDrive;
    private final DcMotorEx frontRightDrive;
    private final DcMotorEx backLeftDrive;
    private final DcMotorEx backRightDrive;
    private final IMU imu;

    private double maxSpeed = 1.0;

    public MecanumDrive(RobotHardware hardware) {
        this.frontLeftDrive = hardware.frontLeftDrive;
        this.frontRightDrive = hardware.frontRightDrive;
        this.backLeftDrive = hardware.backLeftDrive;
        this.backRightDrive = hardware.backRightDrive;
        this.imu = hardware.imu;
    }

    /**
     * Set the maximum speed multiplier (0.0 to 1.0).
     * Use this to limit speed for outreaches or young drivers.
     */
    public void setMaxSpeed(double speed) {
        this.maxSpeed = Math.max(0, Math.min(1, speed));
    }

    /**
     * Robot-relative mecanum drive.
     * @param forward Power in the forward direction (-1 to 1)
     * @param right Power in the right strafe direction (-1 to 1)
     * @param rotate Power for rotation (-1 to 1, positive = clockwise)
     */
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setVelocity((maxSpeed * (frontLeftPower / maxPower)) * RobotConstants.MAX_DRIVE_VELOCITY);
        frontRightDrive.setVelocity((maxSpeed * (frontRightPower / maxPower)) * RobotConstants.MAX_DRIVE_VELOCITY);
        backLeftDrive.setVelocity((maxSpeed * (backLeftPower / maxPower)) * RobotConstants.MAX_DRIVE_VELOCITY);
        backRightDrive.setVelocity((maxSpeed * (backRightPower / maxPower)) * RobotConstants.MAX_DRIVE_VELOCITY);
    }

    /**
     * Field-relative mecanum drive using IMU heading.
     * @param forward Power in the forward direction relative to field (-1 to 1)
     * @param right Power in the right strafe direction relative to field (-1 to 1)
     * @param rotate Power for rotation (-1 to 1, positive = clockwise)
     */
    public void driveFieldRelative(double forward, double right, double rotate) {
        // Convert direction to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate angle by robot heading
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    /**
     * Turn left at specified power.
     */
    public void turnLeft(double power) {
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }

    /**
     * Turn right at specified power.
     */
    public void turnRight(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    /**
     * Point toward a target angle (relative to current heading).
     * @param angle Target angle in degrees (negative = left, positive = right)
     */
    public void pointToAngle(double angle) {
        double power = Math.min(Math.abs(angle) / 60.0, 0.5);

        if (angle < -1) {
            turnLeft(power);
        } else if (angle > 1) {
            turnRight(power);
        }
    }

    /**
     * Stop all drive motors.
     */
    public void stop() {
        frontLeftDrive.setVelocity(0);
        frontRightDrive.setVelocity(0);
        backLeftDrive.setVelocity(0);
        backRightDrive.setVelocity(0);
    }
}
