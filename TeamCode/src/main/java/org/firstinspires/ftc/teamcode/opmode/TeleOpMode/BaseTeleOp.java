package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.YELLOW;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ShooterController;
import org.firstinspires.ftc.teamcode.util.VisionController;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;

/**
 * Abstract base class for TeleOp modes.
 * Handles all common functionality and provides hooks for customization.
 *
 * Subclasses should:
 * - Implement isRedTeam() to return the team color
 * - Override initExtraHardware() if additional hardware is needed
 * - Override extraLoop() for additional per-loop logic
 */
public abstract class BaseTeleOp extends OpMode {

    // Hardware and controllers
    protected RobotHardware hardware;
    protected MecanumDrive drive;
    protected ShooterController shooter;
    protected VisionController vision;
    protected WaverlyGamepad gp;

    // State
    protected boolean relativeDrive = true;
    protected boolean showControls = false;
    protected boolean ballLiftActive = false;
    protected double ballLiftTimer = 0;
    protected double cycles = 0;

    /**
     * Return true if this is the red team, false for blue.
     */
    protected abstract boolean isRedTeam();

    /**
     * Hook for subclasses to initialize additional hardware.
     * Called at the end of init().
     */
    protected void initExtraHardware() {
        // Default: nothing extra
    }

    /**
     * Hook for subclasses to add extra logic to the main loop.
     * Called at the end of each loop iteration.
     */
    protected void extraLoop() {
        // Default: nothing extra
    }

    @Override
    public void init() {
        // Initialize hardware
        hardware = new RobotHardware();
        hardware.initTeleOp(hardwareMap);

        // Initialize controllers
        drive = new MecanumDrive(hardware);
        shooter = new ShooterController(hardware);
        vision = new VisionController(hardware);

        // Initialize gamepad wrapper
        gp = new WaverlyGamepad(gamepad1);

        // Allow subclasses to initialize extra hardware
        initExtraHardware();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        // Read gamepad
        gp.readButtons();

        // Update vision
        vision.update();

        // Handle LED feedback
        updateLED();

        // Handle driving
        handleDriving();

        // Handle shooter
        handleShooting();

        // Handle intake
        handleIntake();

        // Handle ball lift
        handleBallLift();

        // Handle ball ramp
        handleBallRamp();

        // Handle misc controls
        handleMiscControls();

        // Update telemetry
        updateTelemetry();

        // Allow subclasses to add extra logic
        extraLoop();

        cycles++;
    }

    /**
     * Update LED pattern based on vision state and team.
     */
    protected void updateLED() {
        if (vision.canSeeTower()) {
            if (gp.leftTrigger && vision.isPointingToTower()) {
                hardware.led.setPattern(GREEN);
            } else {
                hardware.led.setPattern(YELLOW);
            }
        } else {
            hardware.led.setPattern(isRedTeam() ? RED : BLUE);
        }
    }

    /**
     * Handle driving controls.
     */
    protected void handleDriving() {
        // Toggle drive mode
        if (gp.dpadUpPressed) {
            relativeDrive = !relativeDrive;
        }

        // Reset yaw
        if (gp.dpadDownPressed) {
            hardware.resetYaw();
        }

        // Drive
        if (relativeDrive) {
            drive.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        } else {
            drive.driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // Auto-aim when left trigger is held
        if (gp.leftTrigger && vision.canSeeTower()) {
            drive.pointToAngle(vision.getAngleToTower());
        }
    }

    /**
     * Handle shooting controls.
     */
    protected void handleShooting() {
        // Toggle shooting
        if (gp.yPressed) {
            shooter.toggleShooting();
        }

        // Use recommended velocity when tower is visible
        if (vision.canSeeTower()) {
            double velocity = vision.getRecommendedVelocity();
            // X button reduces velocity slightly
            if (gp.x) {
                velocity -= RobotConstants.VELOCITY_STEP;
            }
            shooter.setTargetVelocity(velocity);
        }

        // Update shooter motors
        shooter.updateShooter();
    }

    /**
     * Handle intake controls.
     */
    protected void handleIntake() {
        // Toggle intake
        if (gp.aPressed) {
            shooter.toggleIntake();
        }

        // Toggle direction
        if (gp.rightTriggerPressed) {
            shooter.toggleIntakeDirection();
        }

        // Update intake motors
        shooter.updateIntake();
    }

    /**
     * Handle ball lift servo control.
     */
    protected void handleBallLift() {
        // B button activates lift
        if (gp.bPressed) {
            ballLiftActive = true;
            ballLiftTimer = 0;
        }

        if (ballLiftActive) {
            shooter.liftBall();
            // Keep lifted for minimum time or while button held
            if (ballLiftTimer >= 10 && !gp.b) {
                ballLiftActive = false;
            }
            ballLiftTimer++;
        } else {
            shooter.lowerBall();
        }
    }

    /**
     * Handle ball ramp control (X button).
     */
    protected void handleBallRamp() {
        shooter.setRampMode(gp.x);
    }

    /**
     * Handle miscellaneous controls.
     */
    protected void handleMiscControls() {
        // Toggle controls display
        if (gp.dpadLeftPressed) {
            showControls = !showControls;
        }
    }

    /**
     * Update telemetry display.
     */
    protected void updateTelemetry() {
        if (showControls) {
            telemetry.addLine("Controls:");
            telemetry.addLine("Down D-Pad resets Yaw");
            telemetry.addLine("Up D-Pad toggles between robot and field relative");
            telemetry.addLine("Triangle (Y) spins shooter motors");
            telemetry.addLine("Circle (B) launches top ball");
            telemetry.addLine("Square (X) activates ramp");
            telemetry.addLine("X activates intake");
            telemetry.addLine("Left joystick moves robot");
            telemetry.addLine("Right joystick turns the robot");
            telemetry.addLine("Left trigger auto-aims at tower");
            telemetry.addLine("Right Trigger flips intake direction");
        } else {
            telemetry.addLine("Info:");
            telemetry.addLine("Team: " + (isRedTeam() ? "RED" : "BLUE"));
            telemetry.addLine("Robot is in " + (relativeDrive ? "Field Relative" : "Robot Relative") + " driving mode");
            telemetry.addLine("Target shooter power: " + shooter.getTargetVelocity() / 20 + "%");
            telemetry.addLine("Current shooter power: L: " + shooter.getLeftShooterVelocity() / 20.0 +
                    "% R: " + shooter.getRightShooterVelocity() / 20.0 + "%");
            telemetry.addLine("Intake Status: " + (shooter.isEjectMode() ? "Eject" : "Intake"));
            telemetry.addLine("Intake Active: " + shooter.isIntakeActive());
        }

        telemetry.addLine("");
        telemetry.addLine("Press Left Dpad to show " + (showControls ? "Robot Info" : "Robot Controls"));

        // Vision info
        if (vision.canSeeTower()) {
            telemetry.addLine("");
            telemetry.addLine("Vision:");
            telemetry.addData("Tx", vision.getTx());
            telemetry.addData("Ty", vision.getTy());
            telemetry.addLine("Distance to tower: " + (int) vision.getDistanceInches() + " in");
            telemetry.addLine("Recommended velocity: " + (int) vision.getRecommendedVelocity());
        }

        telemetry.addLine("");
        telemetry.addLine("Cycle count: " + (int) cycles);

        telemetry.update();
    }
}
