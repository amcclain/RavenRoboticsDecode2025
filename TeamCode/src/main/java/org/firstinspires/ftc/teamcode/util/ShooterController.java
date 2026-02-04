package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controller for the shooter, intake, and ball lift mechanisms.
 * Centralizes all ball handling logic.
 */
public class ShooterController {

    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final DcMotorEx intake;
    private final DcMotorEx belt;
    private final Servo ballLift;
    private final Servo ballRamp;

    private double targetVelocity = RobotConstants.DEFAULT_SHOOTER_VELOCITY;
    private boolean shooting = false;
    private boolean intakeActive = false;
    private int intakeDirection = 1;  // 1 = intake, -1 = eject

    public ShooterController(RobotHardware hardware) {
        this.rightShooter = hardware.rightShooter;
        this.leftShooter = hardware.leftShooter;
        this.intake = hardware.intake;
        this.belt = hardware.belt;
        this.ballLift = hardware.ballLift;
        this.ballRamp = hardware.ballRamp;
    }

    // ==================== Shooter Methods ====================

    /**
     * Toggle shooting on/off.
     */
    public void toggleShooting() {
        shooting = !shooting;
    }

    /**
     * Set shooting state explicitly.
     */
    public void setShooting(boolean active) {
        shooting = active;
    }

    /**
     * Check if shooter is active.
     */
    public boolean isShooting() {
        return shooting;
    }

    /**
     * Set the target shooter velocity.
     */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = Math.max(0, Math.min(velocity, RobotConstants.MAX_SHOOTER_VELOCITY));
    }

    /**
     * Get the target shooter velocity.
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Increase shooter velocity by one step.
     */
    public void increaseVelocity() {
        targetVelocity = Math.min(targetVelocity + RobotConstants.VELOCITY_STEP,
                RobotConstants.MAX_SHOOTER_VELOCITY);
    }

    /**
     * Decrease shooter velocity by one step.
     */
    public void decreaseVelocity() {
        targetVelocity = Math.max(targetVelocity - RobotConstants.VELOCITY_STEP, 0);
    }

    /**
     * Get the current velocity of the left shooter.
     */
    public double getLeftShooterVelocity() {
        return leftShooter.getVelocity();
    }

    /**
     * Get the current velocity of the right shooter.
     */
    public double getRightShooterVelocity() {
        return rightShooter.getVelocity();
    }

    /**
     * Update shooter motors based on current state.
     * Call this in the loop.
     */
    public void updateShooter() {
        if (shooting) {
            rightShooter.setVelocity(targetVelocity);
            leftShooter.setVelocity(targetVelocity);
        } else {
            rightShooter.setVelocity(0);
            leftShooter.setVelocity(0);
        }
    }

    /**
     * Set shooter velocity directly (for autonomous).
     */
    public void setShooterVelocity(double velocity) {
        rightShooter.setVelocity(velocity);
        leftShooter.setVelocity(velocity);
    }

    /**
     * Stop the shooter motors.
     */
    public void stopShooter() {
        rightShooter.setPower(0);
        leftShooter.setPower(0);
        shooting = false;
    }

    // ==================== Intake Methods ====================

    /**
     * Toggle intake on/off.
     */
    public void toggleIntake() {
        intakeActive = !intakeActive;
    }

    /**
     * Set intake state explicitly.
     */
    public void setIntakeActive(boolean active) {
        intakeActive = active;
    }

    /**
     * Check if intake is active.
     */
    public boolean isIntakeActive() {
        return intakeActive;
    }

    /**
     * Toggle intake direction (intake/eject).
     */
    public void toggleIntakeDirection() {
        intakeDirection = -intakeDirection;
    }

    /**
     * Check if intake is in eject mode.
     */
    public boolean isEjectMode() {
        return intakeDirection < 0;
    }

    /**
     * Update intake and belt motors based on current state.
     * Call this in the loop.
     */
    public void updateIntake() {
        if (intakeActive) {
            intake.setVelocity(intakeDirection * RobotConstants.MAX_DRIVE_VELOCITY);
            belt.setVelocity(intakeDirection * RobotConstants.MAX_DRIVE_VELOCITY);
        } else {
            intake.setVelocity(0);
            belt.setVelocity(0.5 * RobotConstants.MAX_DRIVE_VELOCITY);  // Slight belt movement when idle
        }
    }

    /**
     * Spin intake at full speed (for autonomous).
     */
    public void spinIntake() {
        intake.setPower(1);
        belt.setPower(0.8);
    }

    /**
     * Stop the intake and belt.
     */
    public void stopIntake() {
        intake.setPower(0);
        belt.setPower(0);
        intakeActive = false;
    }

    /**
     * Set belt to specific power (for ramp operations).
     */
    public void setBeltPower(double power) {
        belt.setPower(power);
    }

    /**
     * Set belt and intake to specific velocities (for X button hold).
     */
    public void setRampMode(boolean active) {
        if (active) {
            belt.setVelocity(0.75 * RobotConstants.MAX_DRIVE_VELOCITY);
            intake.setVelocity(0.6 * RobotConstants.MAX_DRIVE_VELOCITY);
            if (ballRamp != null) {
                ballRamp.setPosition(RobotConstants.BALL_RAMP_UP);
            }
        } else {
            if (ballRamp != null) {
                ballRamp.setPosition(RobotConstants.BALL_RAMP_REST);
            }
        }
    }

    // ==================== Ball Lift Methods ====================

    /**
     * Lift the ball (set servo to up position).
     */
    public void liftBall() {
        if (ballLift != null) {
            ballLift.setPosition(RobotConstants.BALL_LIFT_UP);
        }
    }

    /**
     * Lower the ball (set servo to rest position).
     */
    public void lowerBall() {
        if (ballLift != null) {
            ballLift.setPosition(RobotConstants.BALL_LIFT_REST);
        }
    }

    /**
     * Set ball lift to autonomous up position.
     */
    public void liftBallAuto() {
        if (ballLift != null) {
            ballLift.setPosition(RobotConstants.BALL_LIFT_AUTO_UP);
        }
    }

    /**
     * Set ball ramp position.
     */
    public void setBallRampPosition(double position) {
        if (ballRamp != null) {
            ballRamp.setPosition(position);
        }
    }

    // ==================== Autonomous Sequence Methods ====================

    /**
     * Unload all balls (autonomous routine).
     * NOTE: This method contains sleep() calls and should only be used in LinearOpMode.
     * @param power Shooter power (0 to 1)
     * @param sleepFunction Lambda to call Thread.sleep or opMode.sleep
     */
    public void unloadBalls(double power, SleepFunction sleepFunction) throws InterruptedException {
        // Spin up motors
        setShooterVelocity(RobotConstants.MAX_DRIVE_VELOCITY * power);

        // Turn belt on
        belt.setPower(0.5);
        sleepFunction.sleep(1250);

        // Shoot ball 1
        liftBallAuto();
        sleepFunction.sleep(1000);
        lowerBall();

        // Turn on belt and intake
        spinIntake();
        sleepFunction.sleep(1750);

        // Shoot ball 2
        liftBallAuto();
        sleepFunction.sleep(1000);
        lowerBall();
        sleepFunction.sleep(1250);

        // Shoot ball 3
        liftBallAuto();
        sleepFunction.sleep(1000);
        lowerBall();

        // Stop everything
        stopIntake();
        stopShooter();
    }

    /**
     * Functional interface for sleep calls.
     */
    @FunctionalInterface
    public interface SleepFunction {
        void sleep(long milliseconds) throws InterruptedException;
    }
}
