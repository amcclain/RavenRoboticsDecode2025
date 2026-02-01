package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.util.List;

@Autonomous(name = "Testing Suit", group = "robot")
@Disabled
public class Testing_Suite extends LinearOpMode{

    //declare DriveBase motors
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    //Declare Ball Magazine Servos and motors
    DcMotor intake, belt;
    Servo liftServo;

    //declare Shooter motors and servos
    DcMotorEx rightShooterMotor, leftShooterMotor;

    WaverlyGamepad wgp;

    double adjustableInches = 24;
    double adjustableDegrees = 45;

    private AprilTagProcessor aprilTag;

    IMU imu;
    boolean onRedTeam;
    String ballOrder;
    double countsPerInch = 29.8;
    double countsPerDegree = 7.4;

    @Override
    public void runOpMode(){
        //define motors
        defineMotors();

        //init IMU
        initIMU();

        //init camera
        initCamera();

        //change values before start
        if (wgp == null){
            wgp = new WaverlyGamepad(gamepad1);
        }
        while (!isStarted()){
            wgp.readButtons();

            //adjust counts per inch
            if (wgp.dpadUpPressed){
                countsPerInch++;
            } else if (wgp.dpadDownPressed){
                countsPerInch--;
            }

            //fine tune counts per inch
            if (wgp.dpadLeftPressed) {
                countsPerInch += 0.1;
            } else if (wgp.dpadRightPressed){
                countsPerInch -= 0.1;
            }

            //adjust counts per degree
            if (wgp.yPressed) {
                countsPerDegree++;
            } else if (wgp.aPressed){
                countsPerDegree--;
            }

            //fine tune counts per degree
            if (wgp.xPressed) {
                countsPerDegree += 0.1;
            } else if (wgp.bPressed){
                countsPerDegree -= 0.1;
            }

            telemetry.addLine("Dpad is counts per inch");
            telemetry.addLine("Shape buttons is counts per degree");
            telemetry.addLine("");
            telemetry.addLine("Up and down is regular tuning");
            telemetry.addLine("Left and right is fine tuning");
            telemetry.addLine("");
            telemetry.addLine("counts per inch is: " + countsPerInch);
            telemetry.addLine("counts per degree is: " + countsPerDegree);
            telemetry.update();
        }


        waitForStart();
//--------------------------------------------------------------------------------------------------
//      Auto starts
//--------------------------------------------------------------------------------------------------

        shootBalls();

    }


    //DriveBase functions
    public void drive(String direction, double power, double inches){
        double targetCounts = inches * countsPerInch,
                flTarg = targetCounts,
                frTarg = targetCounts,
                blTarg = targetCounts,
                brTarg = targetCounts;

        switch (direction) {
            case "back":
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
                //do nothing
                break;
        }

        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition((int) flTarg);
        frontRightDrive.setTargetPosition((int) frTarg);
        backLeftDrive.setTargetPosition((int) blTarg);
        backRightDrive.setTargetPosition((int) brTarg);

        frontLeftDrive.setMode(RUN_TO_POSITION);
        frontRightDrive.setMode(RUN_TO_POSITION);
        backLeftDrive.setMode(RUN_TO_POSITION);
        backRightDrive.setMode(RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(power);

    }
    public void turn(String direction, double power, double degrees){
        double targetCounts = degrees * countsPerDegree,
                flTarg = targetCounts,
                frTarg = -targetCounts,
                blTarg = targetCounts,
                brTarg = -targetCounts;

        if (direction.equals("left")){
            flTarg *= -1;
            frTarg *= -1;
            blTarg *= -1;
            brTarg *= -1;
        }

        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition((int) flTarg);
        frontRightDrive.setTargetPosition((int) frTarg);
        backLeftDrive.setTargetPosition((int) blTarg);
        backRightDrive.setTargetPosition((int) brTarg);

        frontLeftDrive.setMode(RUN_TO_POSITION);
        frontRightDrive.setMode(RUN_TO_POSITION);
        backLeftDrive.setMode(RUN_TO_POSITION);
        backRightDrive.setMode(RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(power);

    }
    private void turnTo(double degrees){
        turn("left", 0.5, degrees - getHeader());
        int isDone = 0;
        boolean wasDone = false;
        while (true) {
            double currentAngle = getHeader();
            double turnPower = Math.min(Math.abs(degrees) / 40, 0.25);
            if (currentAngle <= degrees - 0.5) {
                turnLeft(turnPower / (isDone + 1));
                wasDone = false;
            } else if (currentAngle >= degrees + 0.5) {
                turnRight(turnPower / (isDone + 1));
                wasDone = false;
            } else {
                if (!wasDone) {
                    isDone++;
                }
                wasDone = true;

                if (isDone > 1) break;
            }
            telemetry.addLine("isDone: " + isDone);
            telemetry.addLine("wasDone: " + wasDone);
            telemetry.update();
        }
    }
    public void move(double tilesX, double tilesY){

        //convert x and y from tiles to inches
        double inchesX = tilesToInches(tilesX);
        double inchesY = tilesToInches(tilesY);

        //convert to polar coordinates
        double theta = Math.atan2(inchesX, inchesY);
        double r = Math.hypot(inchesX, inchesY);

        //rotate to orient relative to field
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        //convert back to cartesian coordinates
        double newInchesX = r * Math.cos(theta);
        double newInchesY = r * Math.sin(theta);


        double
                //calculate target counts for x
                xTargetCounts = newInchesX * countsPerInch,

                //calculate target counts for y
                yTargetCounts = newInchesY * countsPerInch,

                //calculate final target counts for motors
                flTarget = yTargetCounts + xTargetCounts,
                frTarget = yTargetCounts - xTargetCounts,
                blTarget = yTargetCounts - xTargetCounts,
                brTarget = yTargetCounts + xTargetCounts;

        //run the motors to the calculated target positions
        runMotorsToPos(flTarget, frTarget, blTarget, brTarget, 0.5);

    }
    public void pickUpBalls(){
        if (onRedTeam){
            turnTo(90);
        } else {
            turnTo(-90);
        }

        spinIntake();

        drive("forward", 0.2, 24);

        drive("backward", 0.2, 24);

        turnTo(0);

        stopIntake();
    }


    //Intake functions
    public void spinIntake(){
        intake.setPower(1);
        belt.setPower(0.8);
    }
    public void stopIntake(){
        intake.setPower(0);
        belt.setPower(0);
    }


    //shooting functions
    private void unloadBalls(double power){
        //spin up motor
        rightShooterMotor.setPower(power);
        leftShooterMotor.setPower(power);
        sleep(250);

        //shoot ball
        liftServo.setPosition(0.3);
        sleep(100);
        liftServo.setPosition(0.3);

        //turn on belt
        belt.setPower(0.8);
        sleep(100);

        //shoot ball
        liftServo.setPosition(0.3);
        sleep(100);
        liftServo.setPosition(0.3);
        sleep(100);

        //shoot ball
        liftServo.setPosition(0.3);
        sleep(100);
        liftServo.setPosition(0.3);

        //turn off belt
        belt.setPower(0.8);

        //spin down motor
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);

    }
    private void shootBalls(){
        pointToTower();
        unloadBalls(getPowerToShoot());
    }


    //util functions
    private double getHeader(){
        //asks imu for yaw pitch and roll and then ignores everything but yaw
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private void pointToTower(){
        while (true) {
            double angleToTower = getAngleToTower();
            double turnPower = Math.min(Math.abs(angleToTower) / 40, 0.5);
            if (angleToTower < -1) {
                turnLeft(turnPower);
                sleep(1);
            } else if (angleToTower > 1) {
                turnRight(turnPower);
                sleep(1);
            } else {
                break;
            }
        }
    }
    private void resetHeading(){
        //resets the IMU yaw
        imu.resetYaw();
    }
    public void turnRight(double power){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }
    public void turnLeft(double power){
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }
    private double tilesToInches(double tiles){
        double inches;
        inches = tiles * 24;
        return inches;
    }
    public void runMotorsToPos(double frontLeftTarget, double frontRightTarget, double backLeftTarget, double backRightTarget, double power){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);

        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition((int) frontLeftTarget);
        frontRightDrive.setTargetPosition((int) frontRightTarget);
        backLeftDrive.setTargetPosition((int) backLeftTarget);
        backRightDrive.setTargetPosition((int) backRightTarget);

        frontLeftDrive.setMode(RUN_TO_POSITION);
        frontRightDrive.setMode(RUN_TO_POSITION);
        backLeftDrive.setMode(RUN_TO_POSITION);
        backRightDrive.setMode(RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(power);
    }


    //camera functions
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
    private double getAngleToTower(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection tag : currentDetections){
            if (!onRedTeam && tag.id == 20) {
                return tag.ftcPose.bearing;
            }
            if (onRedTeam && tag.id == 24) {
                return tag.ftcPose.bearing;
            }
        }
        return 0;
    }
    private double getPowerToShoot(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double dist = 0;
        for (AprilTagDetection tag : currentDetections){
            if (!onRedTeam && tag.id == 20) {
                dist = tag.ftcPose.range;
            }
            if (onRedTeam && tag.id == 24) {
                dist = tag.ftcPose.range;
            }
        }

        return 0.129165 * dist + 39.56322;
    }


    //functions to make code look better
    public void Wait(long duration){
        //just to make code more readable
        sleep(duration);
    }


    //init functions
    public void defineMotors(){
        //define DriveBase motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightMotor");

        //defines Shooter motors
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");

        //defines Ball Magazine Servos and motors
        intake = hardwareMap.get(DcMotor.class, "Intake");
        belt = hardwareMap.get(DcMotor.class, "Belt");
        liftServo = hardwareMap.get(Servo.class, "BallLiftA");

        //reverses motors that are on backwards
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
    }
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
        VisionPortal visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

        telemetry.addLine("Camera working");
        telemetry.update();
    }
    public void initIMU(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }
}