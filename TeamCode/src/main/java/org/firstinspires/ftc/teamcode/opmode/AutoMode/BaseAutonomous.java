package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.MecanumDriveAuto;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ShooterController;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;

/**
 * Abstract base class for Autonomous modes.
 * Handles hardware initialization and provides common drive/shooting methods.
 *
 * Subclasses should:
 * - Implement getCountsPerInch() for encoder tuning
 * - Implement getCountsPerDegree() for encoder tuning
 * - Implement runAutonomousRoutine() with the actual autonomous sequence
 */
public abstract class BaseAutonomous extends LinearOpMode {

    // Hardware and controllers
    protected RobotHardware hardware;
    protected MecanumDriveAuto drive;
    protected ShooterController shooter;
    protected WaverlyGamepad wgp;

    // Tuning values (can be adjusted during pre-start)
    protected double adjustableInches = 24;
    protected double adjustableDegrees = 90;

    /**
     * Return the encoder counts per inch for this autonomous mode.
     */
    protected abstract double getCountsPerInch();

    /**
     * Return the encoder counts per degree for this autonomous mode.
     */
    protected abstract double getCountsPerDegree();

    /**
     * Return true if this is the red team, false for blue.
     */
    protected abstract boolean isRedTeam();

    /**
     * Run the autonomous routine.
     * Implement this with the specific movements and actions for this autonomous.
     */
    protected abstract void runAutonomousRoutine() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        hardware = new RobotHardware();
        hardware.initAutonomous(hardwareMap);

        // Initialize drive controller
        drive = new MecanumDriveAuto(hardware);
        drive.setCountsPerInch(getCountsPerInch());
        drive.setCountsPerDegree(getCountsPerDegree());

        // Initialize shooter controller
        shooter = new ShooterController(hardware);

        // Initialize gamepad wrapper
        wgp = new WaverlyGamepad(gamepad1);

        // Pre-start tuning loop
        preStartTuning();

        // Wait for start
        waitForStart();

        // Reset heading
        drive.resetHeading();

        // Run the autonomous routine
        runAutonomousRoutine();
    }

    /**
     * Pre-start tuning loop allows adjusting encoder values before match.
     */
    protected void preStartTuning() {
        while (!isStarted() && !isStopRequested()) {
            wgp.readButtons();

            // Change inches by 1 (coarse)
            if (wgp.dpadUpPressed) {
                adjustableInches++;
            } else if (wgp.dpadDownPressed) {
                adjustableInches--;
            }

            // Change inches by 1 (fine - same for now)
            if (wgp.dpadLeftPressed) {
                adjustableInches++;
            } else if (wgp.dpadRightPressed) {
                adjustableInches--;
            }

            // Change degrees by 15 (coarse)
            if (wgp.yPressed) {
                adjustableDegrees += 15;
            } else if (wgp.aPressed) {
                adjustableDegrees -= 15;
            }

            // Change degrees by 1 (fine)
            if (wgp.xPressed) {
                adjustableDegrees++;
            } else if (wgp.bPressed) {
                adjustableDegrees--;
            }

            // Display tuning info
            telemetry.addLine("=== Pre-Start Tuning ===");
            telemetry.addLine("Team: " + (isRedTeam() ? "RED" : "BLUE"));
            telemetry.addLine("");
            telemetry.addLine("Dpad is counts per inch");
            telemetry.addLine("Shape buttons is counts per degree");
            telemetry.addLine("");
            telemetry.addLine("Up/Down is regular tuning");
            telemetry.addLine("Left/Right is fine tuning");
            telemetry.addLine("");
            telemetry.addLine("Counts per inch: " + getCountsPerInch());
            telemetry.addLine("Counts per degree: " + getCountsPerDegree());
            telemetry.addLine("");
            telemetry.addLine("Adjustable inches: " + adjustableInches);
            telemetry.addLine("Adjustable degrees: " + adjustableDegrees);
            telemetry.addLine("");

            if (hardware.imu != null && hardware.imu.getRobotYawPitchRollAngles() != null) {
                telemetry.addLine("IMU ONLINE");
            }

            telemetry.update();
        }
    }

    // ==================== Convenience Methods ====================

    /**
     * Drive in a direction for a given distance.
     */
    protected void driveDistance(String direction, double power, double inches) {
        drive.drive(direction, power, inches);
    }

    /**
     * Turn in a direction for given degrees.
     */
    protected void turnDegrees(String direction, double power, double degrees) {
        drive.turn(direction, power, degrees);
    }

    /**
     * Wait/sleep for specified milliseconds.
     */
    protected void Wait(long milliseconds) {
        sleep(milliseconds);
    }

    /**
     * Spin intake at full speed.
     */
    protected void spinIntake() {
        shooter.spinIntake();
    }

    /**
     * Stop the intake.
     */
    protected void stopIntake() {
        shooter.stopIntake();
    }

    /**
     * Unload all balls at specified power.
     */
    protected void unloadBalls(double power) throws InterruptedException {
        shooter.unloadBalls(power, this::sleep);
    }

    /**
     * Drive all motors at constant power (for wall alignment).
     */
    protected void driveConstantPower(double power) {
        drive.driveConstantPower(power);
    }

    /**
     * Update telemetry with current state.
     */
    protected void updateTelemetry() {
        telemetry.addLine("");
        telemetry.addLine("Team: " + (isRedTeam() ? "RED" : "BLUE"));
        telemetry.update();
    }
}
