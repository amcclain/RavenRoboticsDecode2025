package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "New Auto", group = "robot")
public class New_Auto extends LinearOpMode{

    //declare DriveBase motors
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    //Declare Ball Magazine Servos and motors
    DcMotor intake, belt;
    Servo liftServo;

    //declare Shooter motors and servos
    DcMotor rightShooterMotor, leftShooterMotor;

    WaverlyGamepad wgp;

    long adjustableTime = 1300;
    double adjustablePower = 0.4;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    boolean onRedTeam;
    String ballOrder;

    @Override
    public void runOpMode() throws InterruptedException {
        //define DriveBase motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightMotor");

        //defines Shooter motors
        rightShooterMotor = hardwareMap.get(DcMotor.class, "RightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotor.class, "LeftShooterMotor");

        //defines Ball Magazine Servos and motors
        intake = hardwareMap.get(DcMotor.class, "Intake");
        belt = hardwareMap.get(DcMotor.class, "Belt");
        liftServo = hardwareMap.get(Servo.class, "BallLiftA");


        //reverses motors that are on backwards
        backLeftDrive.setDirection(REVERSE);
        frontLeftDrive.setDirection(REVERSE);
        frontRightDrive.setDirection(REVERSE);
        rightShooterMotor.setDirection(REVERSE);


        //tells DriveBase motors to run using encoder to me more accurate
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);

        //tells Shooter motors to run using encoder to me more accurate
        rightShooterMotor.setMode(RUN_USING_ENCODER);
        leftShooterMotor.setMode(RUN_USING_ENCODER);

        //initialize the camera
        initCamera();

        //initialize the gamepad
        if (wgp == null){
            wgp = new WaverlyGamepad(gamepad1);
        }

        //
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

            telemetry.addLine("adjustable time is: " + adjustableTime);
            telemetry.addLine("adjustable power is: " + adjustablePower);
            telemetry.update();
        }


        waitForStart();
//--------------------------------------------------------------------------------------------------
//      Auto starts
//--------------------------------------------------------------------------------------------------

        //belt.setPower(0.8);

        Wait(500);

        driveBackward(0.5, 1000);

        Wait(100);

        shootBall(0.4);

        shootBall(0.4);

        shootBall(0.4);

        readTeam();

        turn(-0.5, 300);

        Wait(100);

        readOrder();

        turn(-0.5, 300);

        driveBackward(0.5, 650);


    }


    //DriveBase functions
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

    //Non-team specific DriveBase functions
    public void strafe(double power, long duration){
        if (onRedTeam) {
            driveRight(power, duration);
        } else {
            driveLeft(power, duration);
        }
    }
    public void turn(double power, long duration){
        if (onRedTeam) {
            turnRight(power, duration);
        } else {
            turnLeft(power, duration);
        }
    }


    //Shooter and Ball Magazine functions
    public void shootBall(double power){
        rightShooterMotor.setPower(power);
        leftShooterMotor.setPower(power);
        belt.setPower(0.8);
        sleep(1250);
        liftServo.setPosition(0.3);
        sleep(1250);
        belt.setPower(0);
        liftServo.setPosition(0);
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);
        sleep(1250);
    }
    public void spinIntake(){
        intake.setPower(1);
        belt.setPower(0.8);
    }
    public void stopIntake(){
        intake.setPower(0);
        belt.setPower(0);
    }


    //camera functions
    private void initCamera() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the webcam
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private void readTeam(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == 20)
                onRedTeam = false;
            else if (tag.id == 24)
                onRedTeam = true;
        }
    }
    private void readOrder(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == 21)
                ballOrder = "GPP";
            else if (tag.id == 22)
                ballOrder = "PGP";
            else if (tag.id == 23)
                ballOrder = "PPG";
        }
    }


    //functions to make code look better
    public void Wait(long duration){
        //just to make code more readable
        sleep(duration);
    }
}