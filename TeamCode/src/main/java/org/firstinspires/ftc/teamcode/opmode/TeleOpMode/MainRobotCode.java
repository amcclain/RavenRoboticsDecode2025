package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.YELLOW;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math;
import java.util.List;


@TeleOp(name = "Red Main", group = "Robot")
public class MainRobotCode extends OpMode {

    //declares the motors and servos
    DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotorEx intake, belt;
    DcMotorEx rShooter, lShooter;
    Servo ballLift, ballRamp;
    RevBlinkinLedDriver led;
    WaverlyGamepad gp = null;

    //camera stuff
    private AprilTagProcessor aprilTag;

    //declares the Inertial Measurement Unit
    private IMU imu;

    boolean redTeam = true;
    boolean autoShooting = false;

    // Set from tag detection
    double distToRedTower;
    double angleToRedTower;
    double distToBlueTower;
    double angleToBlueTower;
    String ballOrder = "UNKNOWN";

    double getDistToTower() {
        return this.redTeam ? this.distToRedTower : this.distToBlueTower;
    }
    double recVelocity;

    double cycles = 0;
    boolean canSeeTower = false;
    boolean pointingToTower = false;

    double velocity = 1000;
    int maxVelocity = 2000;
    int stepVelocitySize = 1;
    int stepVelocity = 20 * stepVelocitySize;
    boolean shooting = false;

    int direction = 1;
    boolean relativeDrive = true;
    boolean intakeActive = false;
    boolean showControls = false;
    boolean aLifting = false;
    boolean bLifting = false;
    double timerA = 0;
    double timerB = 0;

    private Limelight3A limelight;

    @Override
    public void init() {

        //define DcMotors
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BackRight");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        belt = hardwareMap.get(DcMotorEx.class, "Belt");

        //define DcMotorExs
        rShooter = hardwareMap.get(DcMotorEx.class, "RightShooter");
        lShooter = hardwareMap.get(DcMotorEx.class, "LeftShooter");

        //define servos
        ballLift = hardwareMap.get(Servo.class, "BallLift");
        ballRamp = hardwareMap.get(Servo.class, "BallRamp");

        //define very cool LED
        led = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        //flips the direction of the necessary motors
        backRightDrive.setDirection(REVERSE);
        backLeftDrive.setDirection(REVERSE);
        rShooter.setDirection(REVERSE);

        //tells motors to use RUN_USING_ENCODER to be more accurate
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);
        rShooter.setMode(RUN_USING_ENCODER);
        lShooter.setMode(RUN_USING_ENCODER);

        gp = new WaverlyGamepad(gamepad1);

        //setting up the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void start(){
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    // All units must match (inches recommended)
    static final double CAMERA_HEIGHT_IN = 10;        // your robot
    static final double TAG_HEIGHT_IN = 29.5;         // center of AprilTag
    static final double CAMERA_PITCH_DEG = 22.5;      // Limelight tilt

    @Override
    public void loop() {
        // limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            if (llResult.getStaleness() < 50)
                canSeeTower = true;
            else canSeeTower = false;

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            // -----------------------------
            // DISTANCE TO APRILTAG (WORKING METHOD)
            // -----------------------------
            double tx = llResult.getTx();
            double ty = llResult.getTy();

            double angleToTagDeg = CAMERA_PITCH_DEG + ty;
            double angleToTagRad = Math.toRadians(angleToTagDeg);

            double distanceInches =
                    (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN)
                            / Math.tan(angleToTagRad);

            telemetry.addLine("Dist to AprilTag (in): " + distanceInches);
            telemetry.addLine("");

            recVelocity = (maxVelocity/100d) * calcRecVelocityPct(distanceInches);
            telemetry.addLine("Recommended power in order to score is: " + recVelocity / stepVelocity + "%");
            telemetry.addLine("");

            telemetry.addLine("Angle to tower is: " + tx);
            telemetry.addLine("");

            if (gp.leftTrigger){
                pointToTower(tx);
            }

            if (tx < -4) pointingToTower = false;
            else if (tx > 4) pointingToTower = false;
            else pointingToTower = true;
        }



        //add telemetry
        if (showControls) {
            telemetry.addLine("Controls:");
            telemetry.addLine("Down D-Pad resets Yaw");
            telemetry.addLine("Up D-Pad toggles between robot and field relative");
            telemetry.addLine("Triangle spins shooter motors");
            telemetry.addLine("Circle launches top ball");
            telemetry.addLine("Square launches all balls");
            telemetry.addLine("X activates intake");
            telemetry.addLine("Left joystick moves robot");
            telemetry.addLine("Right joystick turns the robot");
            telemetry.addLine("Left and right bumpers increase and decrease the shooting power by " + stepVelocitySize + "%");
            telemetry.addLine("Right Trigger flips intake direction");
        } else {
            telemetry.addLine("Info:");
            telemetry.addLine("Robot is in " + (relativeDrive ? "Field Relative" : "Robot Relative") + " driving mode");
            telemetry.addLine("Target shooter power: " + velocity / 20 + "%");
            telemetry.addLine("Current shooter power: Left: " + lShooter.getVelocity() / 20d + "% Right: " + rShooter.getVelocity() / 20d + "%");
            telemetry.addLine("Intake Status: " + (direction == 1 ? "Intake" : "Eject"));
            telemetry.addLine("Intake Active: " + intakeActive);
            telemetry.addLine("Auto power level active: " + autoShooting);
        }
        telemetry.addLine("");
        telemetry.addLine("Press Left Dpad to show " + (showControls? "Robot Info" : "Robot Controls"));
        telemetry.addLine("");
        telemetry.addLine("ticks motors have turned:");
        telemetry.addLine("FL: " + frontLeftDrive.getVelocity() + "FR: " + frontRightDrive.getVelocity() + "BL: " + backLeftDrive.getVelocity() + "BR: " + backRightDrive.getVelocity());
        telemetry.addLine("");
        telemetry.addLine("Correct order of balls: " + ballOrder);
        telemetry.addLine("");
        telemetry.addLine("Distance to the " + (redTeam? "red" : "blue") + " tower: " + (int) getDistToTower());
        telemetry.addLine("");


        gp.readButtons();


        //LEDs
        if (canSeeTower)
            if (gp.leftTrigger && pointingToTower)
                led.setPattern(GREEN);
            else
                led.setPattern(YELLOW);
        else {
            if (redTeam)
                led.setPattern(RED);
            else
                led.setPattern(BLUE);
        }



        //reset robot Yaw
        if (gp.dpadDownPressed){
            imu.resetYaw();
        }


        //intake
        if (gp.aPressed){
            intakeActive = !intakeActive;
        }
        if (gp.rightTriggerPressed){
            direction = -direction;
        }
        if (intakeActive) {
            intake.setVelocity(direction*2000);
            belt.setVelocity(direction*2000);
        } else {
            intake.setVelocity(0*2000);
            belt.setVelocity(0.5*2000);
        }


        //shooting
        if (gp.yPressed){
            shooting = !shooting;
        }
        if (canSeeTower) {
            velocity = recVelocity;
            if (gp.x){
                velocity -= 20;
            }
        }
        if (shooting){
            rShooter.setVelocity(velocity);
            lShooter.setVelocity(velocity);
        } else {
            rShooter.setVelocity(0);
            lShooter.setVelocity(0);
        }


        //Lift servos
        if (gp.bPressed) {
            aLifting = true;
            timerA = 0;
        }

        if (aLifting){

            ballLift.setPosition(0.12+0.06);

            if (timerA >= 10 && !gp.b)
                aLifting = false;

            timerA++;
        } else {
            ballLift.setPosition(0.06);
        }

        if (gp.x){
            ballRamp.setPosition(0.13);
            belt.setVelocity(0.75*2000);
            intake.setVelocity(0.5*2000);
        } else {
            ballRamp.setPosition(0.07);
        }
        /*
        if (gp.xPressed){
            aLifting = true;
            bLifting = true;
            timerA = 0;
            timerB = 0;
        }

        if (bLifting){

            ballRamp.setPosition(0.13);
            belt.setVelocity(0.85*2000);

            if (timerB >= 20)
                intake.setVelocity(0.4*2000);

            if (timerB >= 180)
                bLifting = false;

            timerB++;
        } else {
            ballRamp.setPosition(0.07);
        }
        */


        cycles++;
        telemetry.addLine("");
        telemetry.addLine("cycle count is at " + cycles);


        //driving
        if (gp.dpadUpPressed){
            relativeDrive = !relativeDrive;
        }
        if (relativeDrive) {
            drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        } else {
            driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        telemetry.update();

    }


    //Field relative drive
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    //traditional drive
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setVelocity((maxSpeed * (frontLeftPower / maxPower))*2000);
        frontRightDrive.setVelocity((maxSpeed * (frontRightPower / maxPower))*2000);
        backLeftDrive.setVelocity((maxSpeed * (backLeftPower / maxPower))*2000);
        backRightDrive.setVelocity((maxSpeed * (backRightPower / maxPower))*2000);
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the webcam
        CameraName limeLight = hardwareMap.get(WebcamName.class, "limelight");
        builder.setCamera(limeLight);

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        VisionPortal visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private void detectTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        canSeeTower = false;
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == 20) {
                distToBlueTower = tag.ftcPose.range;
                angleToBlueTower = tag.ftcPose.bearing;
                if (!redTeam)
                    canSeeTower = true;
            } else if (tag.id == 24) {
                distToRedTower = tag.ftcPose.range;
                angleToRedTower = tag.ftcPose.bearing;
                if (redTeam)
                    canSeeTower = true;
            } else if (tag.id == 21) {
                ballOrder = "GPP";
            } else if (tag.id == 22) {
                ballOrder = "PGP";
            } else if (tag.id == 23) {
                ballOrder = "PPG";
            }
        }
    }

    private double calcRecVelocityPct(double distance){
        double ret;

        //new
        //ret = 0.20847 * distance + 34.25142;

        //new
        ret = 0.260102 * distance + 34.80803;

        //arbitrary add 2
        //ret += 2;

        //make sure power is between 0% and 100%
        ret = Math.max(ret, 0);
        ret = Math.min(ret, 100);

        return ret;
    }

    private void turnLeft(double power){
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }

    private void turnRight(double power){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    private void pointToTower(double angle){
        double power = Math.min(-Math.abs(angle)/60, 0.5);

        if (angle < -1){
            turnLeft(power);
        } else if (angle > 1){
            turnRight(power);
        }
    }

}
