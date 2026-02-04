package org.firstinspires.ftc.teamcode.util;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drive controller for Autonomous mode.
 * Provides encoder-based position control for precise movements.
 */
public class MecanumDriveAuto {

    private final DcMotorEx frontLeftDrive;
    private final DcMotorEx frontRightDrive;
    private final DcMotorEx backLeftDrive;
    private final DcMotorEx backRightDrive;
    private final IMU imu;

    private double countsPerInch;
    private double countsPerDegree;

    public MecanumDriveAuto(RobotHardware hardware) {
        this.frontLeftDrive = hardware.frontLeftDrive;
        this.frontRightDrive = hardware.frontRightDrive;
        this.backLeftDrive = hardware.backLeftDrive;
        this.backRightDrive = hardware.backRightDrive;
        this.imu = hardware.imu;

        // Default to close auto values
        this.countsPerInch = RobotConstants.CLOSE_COUNTS_PER_INCH;
        this.countsPerDegree = RobotConstants.CLOSE_COUNTS_PER_DEGREE;
    }

    /**
     * Set the encoder counts per inch for distance calculations.
     */
    public void setCountsPerInch(double countsPerInch) {
        this.countsPerInch = countsPerInch;
    }

    /**
     * Set the encoder counts per degree for rotation calculations.
     */
    public void setCountsPerDegree(double countsPerDegree) {
        this.countsPerDegree = countsPerDegree;
    }

    public double getCountsPerInch() {
        return countsPerInch;
    }

    public double getCountsPerDegree() {
        return countsPerDegree;
    }

    /**
     * Drive in a specified direction for a given distance.
     * @param direction "forward", "backward", "left", or "right"
     * @param power Motor power (0 to 1)
     * @param inches Distance to travel
     */
    public void drive(String direction, double power, double inches) {
        double targetCounts = inches * countsPerInch;
        double flTarg = targetCounts;
        double frTarg = targetCounts;
        double blTarg = targetCounts;
        double brTarg = targetCounts;

        switch (direction) {
            case "backward":
                flTarg *= -1;
                frTarg *= -1;
                blTarg *= -1;
                brTarg *= -1;
                break;
            case "left":
                flTarg *= -1;
                brTarg *= -1;
                break;
            case "right":
                frTarg *= -1;
                blTarg *= -1;
                break;
            default:
                // "forward" - no changes needed
                break;
        }

        runMotorsToPosition(flTarg, frTarg, blTarg, brTarg, power);
    }

    /**
     * Turn in a specified direction for a given number of degrees.
     * @param direction "left" or "right"
     * @param power Motor power (0 to 1)
     * @param degrees Degrees to turn
     */
    public void turn(String direction, double power, double degrees) {
        double targetCounts = degrees * countsPerDegree;
        double flTarg = targetCounts;
        double frTarg = -targetCounts;
        double blTarg = targetCounts;
        double brTarg = -targetCounts;

        if (direction.equals("left")) {
            flTarg *= -1;
            frTarg *= -1;
            blTarg *= -1;
            brTarg *= -1;
        }

        runMotorsToPosition(flTarg, frTarg, blTarg, brTarg, power);
    }

    /**
     * Turn to a specific heading using IMU feedback.
     * @param targetDegrees Target heading in degrees
     */
    public void turnTo(double targetDegrees) {
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);

        while (true) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (currentAngle <= targetDegrees - 0.5) {
                turnLeft(0.1);
            } else if (currentAngle >= targetDegrees + 0.5) {
                turnRight(0.1);
            } else {
                break;
            }
        }

        stop();
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
     * Run all motors to specified encoder positions.
     */
    public void runMotorsToPosition(double flTarget, double frTarget,
                                     double blTarget, double brTarget, double power) {
        // Stop motors and reset encoders
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);

        // Set target positions
        frontLeftDrive.setTargetPosition((int) flTarget);
        frontRightDrive.setTargetPosition((int) frTarget);
        backLeftDrive.setTargetPosition((int) blTarget);
        backRightDrive.setTargetPosition((int) brTarget);

        // Set mode to run to position
        frontLeftDrive.setMode(RUN_TO_POSITION);
        frontRightDrive.setMode(RUN_TO_POSITION);
        backLeftDrive.setMode(RUN_TO_POSITION);
        backRightDrive.setMode(RUN_TO_POSITION);

        // Set power
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    /**
     * Stop all drive motors.
     */
    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Set all motors to run using encoders for velocity control.
     */
    public void setRunUsingEncoder() {
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);
    }

    /**
     * Drive all motors at constant power (used during wall alignment, etc).
     */
    public void driveConstantPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    /**
     * Reset the IMU yaw heading.
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /**
     * Get the current heading in degrees.
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
