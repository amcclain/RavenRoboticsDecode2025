package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Red Auto", group = "robot")
public class Red_Auto extends LinearOpMode{

    //declare DriveBase motors
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    //Declare Ball Magazine Servos and motors
    DcMotor intake, belt;
    Servo liftServo;

    //declare Shooter motors and servos
    DcMotorEx rightShooterMotor, leftShooterMotor;

    WaverlyGamepad wgp;

    double adjustableInches = 24;
    double adjustableDegrees = 90;

    IMU imu;

    boolean onRedTeam = true;
    String ballOrder;
    final double countsPerInch = 29.8;
    final double countsPerDegree = 7.4;

    @Override
    public void runOpMode() throws InterruptedException {
        //define motors
        defineMotors();

        //init IMU
        initIMU();

        //initialize the gamepad
        if (wgp == null){
            wgp = new WaverlyGamepad(gamepad1);
        }

        //adjust values before start
        while (!isStarted()){
            wgp.readButtons();

            //change inches by 6
            if (wgp.dpadUpPressed){
                adjustableInches++;
            } else if (wgp.dpadDownPressed){
                adjustableInches--;
            }

            //change inches by 1
            if (wgp.dpadLeftPressed) {
                adjustableInches++;
            } else if (wgp.dpadRightPressed){
                adjustableInches--;
            }

            //change degrees by 15
            if (wgp.yPressed) {
                adjustableDegrees += 15;
            } else if (wgp.aPressed){
                adjustableDegrees -= 15;
            }

            //change degrees by 1
            if (wgp.xPressed) {
                adjustableDegrees++;
            } else if (wgp.bPressed){
                adjustableDegrees--;
            }

            telemetry.addLine("Dpad is counts per inch");
            telemetry.addLine("Shape buttons is counts per degree");
            telemetry.addLine("");
            telemetry.addLine("Up and down is regular tuning");
            telemetry.addLine("Left and right is fine tuning");
            telemetry.addLine("");
            telemetry.addLine("counts per inch is: " + adjustableInches);
            telemetry.addLine("counts per degree is: " + adjustableDegrees);
            telemetry.addLine("");
            if (imu.getRobotYawPitchRollAngles() != null) telemetry.addLine("IMU ONLINE");
            telemetry.update();
        }


        waitForStart();
//--------------------------------------------------------------------------------------------------
//      Auto starts
//--------------------------------------------------------------------------------------------------

        resetHeading();

        intake.setPower(0.2);
        belt.setPower(0.5);

        Wait(500);

        drive("backward", 0.8, 93);

        Wait(2000);

        frontLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);

        Wait(1000);

        unloadBalls(0.49);

        turn("right", 0.8, 57);

        Wait(750);

        drive("forward", 0.5, 72);

        spinIntake();

        Wait(2500);

        drive("backward", 0.5, 72);

        Wait(2500);

        stopIntake();

        turn("left", 0.8, 60);

        Wait(750);

        intake.setPower(0.2);

        unloadBalls(0.49);

        drive("right", 0.8, 36);

        sleep(2000);

    }



    //DriveBase functions
    public void drive(String direction, double power, double inches){
        double targetCounts = inches * countsPerInch,
                flTarg = targetCounts,
                frTarg = targetCounts,
                blTarg = targetCounts,
                brTarg = targetCounts;

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

        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);

        while (true) {
            telemetry.addLine("entered turning loop");
            double currentAngle = getHeader();
            telemetry.addLine("angle: " + currentAngle);
            double turnPower = Math.min(Math.abs(degrees) / 40, 0.5);
            telemetry.addLine("power: " + turnPower);
            if (currentAngle <= degrees - 0.5) {
                turnLeft(0.1);
            } else if (currentAngle >= degrees + 0.5) {
                turnRight(0.1);
            } else {
                break;
            }
            updateTel();
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
        rightShooterMotor.setVelocity(2000 * power);
        leftShooterMotor.setVelocity(2000 * power);
        telemetry.addLine("spinning up motors at " + power);
        updateTel();

        //turn belt on
        belt.setPower(0.5);
        sleep(1250);

        //shoot ball
        liftServo.setPosition(0.3);
        sleep(1000);
        liftServo.setPosition(0.06);

        //turn on belt
        spinIntake();
        sleep(1750);

        //shoot ball
        liftServo.setPosition(0.3);
        sleep(1000);
        liftServo.setPosition(0.06);
        sleep(1250);

        //shoot ball
        liftServo.setPosition(0.3);
        sleep(1000);
        liftServo.setPosition(0.06);

        //turn off belt
        stopIntake();

        //spin down motor
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);

    }
    /*
    private void shootBalls(double power){
        pointToTower();
        unloadBalls(power);
    }
    */

    //util functions
    private double getHeader(){
        double angle;
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
    /*
    private void pointToTower(){
        while (true) {
            double angleToTower = getAngleToTower();
            double turnPower = Math.min(Math.abs(angleToTower) / 40, 0.5);
            if (angleToTower < -1) {
                turnLeft(turnPower);
            } else if (angleToTower > 1) {
                turnRight(turnPower);
            } else {
                break;
            }
        }
    }
    */
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
    /*
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
        if (ballOrder == null) ballOrder = "unknown";
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
    */


    //functions to make code look better
    public void Wait(long duration){
        //just to make code more readable
        sleep(duration);
    }


    //init functions
    public void defineMotors(){
        //define DriveBase motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");

        //defines Shooter motors
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "RightShooter");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "LeftShooter");

        //defines Ball Magazine Servos and motors
        intake = hardwareMap.get(DcMotor.class, "Intake");
        belt = hardwareMap.get(DcMotor.class, "Belt");
        liftServo = hardwareMap.get(Servo.class, "BallLift");

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
    public void initIMU(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }

    //util functions
    public void updateTel(){
        telemetry.addLine("");
        telemetry.addLine("on red team? " + onRedTeam);
        telemetry.addLine("ball order is " + ballOrder);
        telemetry.update();
    }
}