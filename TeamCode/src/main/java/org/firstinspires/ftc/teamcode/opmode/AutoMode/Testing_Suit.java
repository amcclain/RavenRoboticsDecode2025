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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Testing Suit", group = "robot")
public class Testing_Suit extends LinearOpMode{

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
    IMU imu;
    double countsPerInch = 29.8;
    double countsPerDegree = 8;

    @Override
    public void runOpMode(){
        //define motors
        defineMotors();

        //init IMU
        initIMU();

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

        ///drive("forward", 0.5, 24);

        turn("right", 0.5, 90);
        Wait(5000);

        ///drive("forward", 0.5, 24);

    }


    //Prototype DriveBase functions
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

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition((int) flTarg);
        frontRightDrive.setTargetPosition((int) frTarg);
        backLeftDrive.setTargetPosition((int) blTarg);
        backRightDrive.setTargetPosition((int) brTarg);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition((int) flTarg);
        frontRightDrive.setTargetPosition((int) frTarg);
        backLeftDrive.setTargetPosition((int) blTarg);
        backRightDrive.setTargetPosition((int) brTarg);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(power);

    }


    //Prototype Shooter functions
    public void shootBalls(double power){
        //spin up motor
        rightShooterMotor.setVelocity(2000*power);
        leftShooterMotor.setVelocity(2000*power);
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


    //Functions to make code look better
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

        //reverses DriveBase motors that are on backwards
        frontLeftDrive.setDirection(REVERSE);
        frontRightDrive.setDirection(REVERSE);
        backLeftDrive.setDirection(REVERSE);
        ///backRightDrive.setDirection(REVERSE);

        //tells DriveBase motors to run using encoder to me more accurate
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);

        //defines Shooter motors
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");

        //defines Ball Magazine Servos and motors
        intake = hardwareMap.get(DcMotor.class, "Intake");
        belt = hardwareMap.get(DcMotor.class, "Belt");
        liftServo = hardwareMap.get(Servo.class, "BallLiftA");

        //reverses shooter motors that are on backwards
        rightShooterMotor.setDirection(REVERSE);

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
}