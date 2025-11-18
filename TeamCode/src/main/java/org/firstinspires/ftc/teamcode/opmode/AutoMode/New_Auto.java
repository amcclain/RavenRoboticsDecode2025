package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "New Auto", group = "robot")
public class New_Auto extends LinearOpMode {
    DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, rightShooterMotor, leftShooterMotor;
    DcMotor intake, belt;
    Servo ballLifter;

    //camera stuff
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //declare gamepad
    WaverlyGamepad wgp;

    //declares the Inertial Measurement Unit
    IMU imu;

    //declare adjustable vars
    long adjustableTime = 1300;
    double adjustablePower = 0.4;

    public void driveForward(double power, long duration){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        sleep(duration);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void newDriveForward(int rotations){
        frontLeftDrive.setTargetPosition(rotations);
        frontRightDrive.setTargetPosition(rotations);
        backLeftDrive.setTargetPosition(rotations);
        backRightDrive.setTargetPosition(rotations);
    }
    public void driveBackward(double power, long duration){
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(-power);
        sleep(duration);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void driveLeft(double power, long duration){
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
        sleep(duration);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void driveRight(double power, long duration){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
        sleep(duration);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void turnRight(double power, long duration){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
        sleep(duration);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void turnLeft(double power, long duration){
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
        sleep(duration);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    //define Shooter and Ball Magazine functions
    public void shootBall(double power){
        rightShooterMotor.setPower(power);
        leftShooterMotor.setPower(power);
        intake.setPower(1);
        sleep(1250);
        ballLifter.setPosition(0.3);
        sleep(1250);
        intake.setPower(0);
        ballLifter.setPosition(0);
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);
        sleep(1250);
    }
    public void spinIntake(double power){
        intake.setPower(power);
        belt.setPower(power);
    }
    public void stopIntake(){
        intake.setPower(0);
        belt.setPower(0);
    }

    //functions to make code look better
    public void Wait(long duration){
        //just to make code more readable
        sleep(duration);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //define DriveBase motors
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BackRightMotor");

        //defines Shooter motors
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");

        //defines Ball Magazine Servos and motors
        intake = hardwareMap.get(DcMotor.class, "Intake");
        belt = hardwareMap.get(DcMotor.class, "Belt");
        ballLifter = hardwareMap.get(Servo.class, "BallLiftA");


        //reverses motors that are on backwards
        frontLeftDrive.setDirection(REVERSE);
        rightShooterMotor.setDirection(REVERSE);


        //tells DriveBase motors to run using encoder to me more accurate
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);

        //tells Shooter motors to run using encoder to me more accurate
        rightShooterMotor.setMode(RUN_USING_ENCODER);
        leftShooterMotor.setMode(RUN_USING_ENCODER);

        //setting up the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //initialize april tag
        initAprilTag();


        //define gamepad
        if (wgp == null){
            wgp = new WaverlyGamepad(gamepad1);
        }

        //adjust adjusted vars
        while (!isStarted()){
            wgp.readButtons();

            //change time
            if (wgp.dpadDownPressed) {
                adjustableTime = Math.max(adjustableTime - 100, 0);
            } else if (wgp.dpadUpPressed){
                adjustableTime += 100;
            }

            //change power
            if (wgp.dpadLeftPressed) {
                adjustablePower = Math.max(adjustablePower - 0.05, 0);
            } else if (wgp.dpadRightPressed){
                adjustablePower += 0.05;
            }

            telemetry.addLine("driving time is: " + adjustableTime);
            telemetry.addLine("shoot power is: " + adjustablePower);
            telemetry.update();
        }


        waitForStart();
//--------------------------------------------------------------------------------------------------
//      Auto starts
//--------------------------------------------------------------------------------------------------

        newDriveForward(10);

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
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


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
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }

}
