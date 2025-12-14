package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math;
import java.util.List;


@TeleOp(name = "Red Main", group = "Robot")
public class MainRobotCode extends OpMode {

    //declares the motors and servos
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, intake, belt;
    DcMotorEx rShooter, lShooter;
    Servo ballLift, ballRamp;
    WaverlyGamepad gp = null;

    //camera stuff
    private AprilTagProcessor aprilTag;

    //declares the Inertial Measurement Unit
    IMU imu;

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
    double getAngleToTower() {
        return this.redTeam ? this.angleToRedTower : this.angleToBlueTower;
    }
    double recVelocity;


    @Override
    public void init() {

        //define DcMotors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightMotor");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        belt = hardwareMap.get(DcMotor.class, "Belt");

        //define DcMotorExs
        rShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        lShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");

        //define servos
        ballLift = hardwareMap.get(Servo.class, "BallLift");
        ballRamp = hardwareMap.get(Servo.class, "BallRamp");

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
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        initAprilTag();
    }

    int velocity = 1000;
    int maxVelocity = 2000;
    int stepVelocitySize = 1;
    int stepVelocity = 20 * stepVelocitySize;
    int direction = 1;
    boolean shooting = false;
    boolean relativeDrive = true;
    boolean intakeActive = false;
    boolean showControls = false;
    boolean aLifting = false;
    double timer = 0;

    @Override
    public void loop() {
        //add telemetry
        if (showControls) {
            telemetry.addLine("Controls:");
            telemetry.addLine("Down D-Pad resets Yaw");
            telemetry.addLine("Up D-Pad toggles between robot and field relative");
            telemetry.addLine("Triangle spins shooter motors");
            telemetry.addLine("Circle launches top ball");
            telemetry.addLine("Square launches middle ball");
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
        telemetry.addLine("");


        detectTags();
        recVelocity = (maxVelocity/100d) * calcRecVelocityPct(getDistToTower());

        telemetry.addLine("Correct order of balls: " + ballOrder);
        telemetry.addLine("");
        telemetry.addLine("Distance to the " + (redTeam? "red" : "blue") + " tower: " + (int) getDistToTower());
        telemetry.addLine("");
        telemetry.addLine("Recommended power in order to score is: " + recVelocity / stepVelocity + "%");
        telemetry.addLine("");
        telemetry.addLine("Angle to tower is: " + getAngleToTower());


        gp.readButtons();


        //reset robot Yaw
        if (gp.dpadDownPressed){
            imu.resetYaw();
        }


        //show or hide controls
        if (gp.dpadLeftPressed){
            showControls = !showControls;
        }


        //intake
        if (gp.aPressed){
            intakeActive = !intakeActive;
        }
        if (gp.rightTriggerPressed){
            direction = -direction;
        }
        if (intakeActive) {
            intake.setPower(direction);
            belt.setPower(direction*0.8);
        } else {
            intake.setPower(0);
            belt.setPower(0.5);
        }


        //shooting
        if (gp.leftBumperPressed){
            velocity = Math.max(velocity - stepVelocity, 0);
        }
        if (gp.rightBumperPressed){
            velocity = Math.min(velocity + stepVelocity, maxVelocity);
        }
        if (gp.leftTriggerPressed){
            autoShooting = !autoShooting;
        }
        if (gp.yPressed){
            shooting = !shooting;
        }
        if (autoShooting){
            velocity = (int) Math.round(recVelocity);
            pointToTower();
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
            timer = 0;
        }

        if (aLifting){

            ballLift.setPosition(0.3);

            timer++;

            if (timer >= 20 && !gp.b)
                aLifting = false;

        } else {
            ballLift.setPosition(0.06);
        }

        if (gp.x){
            ballRamp.setPosition(0.3);
        } else {
            ballRamp.setPosition(0);
        }

        telemetry.addLine("");
        telemetry.addLine("servoLifting is " + aLifting);
        telemetry.addLine("lift servo is at " + ballLift.getPosition());


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
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the webcam
        if (hardwareMap.get(WebcamName.class, "Webcam 1") != null) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        else if (hardwareMap.get(WebcamName.class, "Ethernet Device") != null) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Ethernet Device"));
        }
        else {
            throw new IllegalArgumentException("Camera Not Working");
        }


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

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
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == 20) {
                distToBlueTower = tag.ftcPose.range;
                angleToBlueTower = tag.ftcPose.bearing;
            } else if (tag.id == 24) {
                distToRedTower = tag.ftcPose.range;
                angleToRedTower = tag.ftcPose.bearing;
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
        //y=0.20847x+34.25142
        ret = 0.20847 * distance + 34.25142;

        //old
        //ret = 0.164681 * distance + 37.09811;

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

    private void pointToTower(){
        double angle = getAngleToTower();
        double power = Math.min(Math.abs(angle)/40, 0.5);

        if (angle < -1){
            turnLeft(power);
        } else if (angle > 1){
            turnRight(power);
        }
    }

}
