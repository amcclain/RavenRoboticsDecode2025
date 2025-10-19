package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Main Auto Code", group = "robot")
public class MainAutoCode extends LinearOpMode{

    //declare DriveBase motors
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    //Declare Ball Magazine Servos
    Servo LiftServo;
    CRServo Intake, Belt;

    //declare Shooter motors and servos
    DcMotor Rshooter ,Lshooter;
    Servo LeftTilter, RightTilter;

    //define DriveBase functions
    public void DriveForward(double power, long duration){
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
    public void TurnRight(double power, long duration){
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
    public void TurnLeft(double power, long duration){
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
    public void ShootBall(double power){
        Rshooter.setPower(power);
        Lshooter.setPower(power);
        Belt.setPower(1);
        sleep(250);
        LiftServo.setPosition(0.2);
        Belt.setPower(0);
        sleep(100);
        LiftServo.setPosition(0);
        Rshooter.setPower(0);
        Lshooter.setPower(0);
        sleep(100);
    }

    //other functions
    public void Wait(long duration){
        sleep(duration);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //define DriveBase motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        //defines Shooter motors and servos
        Rshooter = hardwareMap.get(DcMotor.class, "Right_shooter_motor");
        Lshooter = hardwareMap.get(DcMotor.class, "Left_shooter_motor");
        LeftTilter = hardwareMap.get(Servo.class, "LeftTilter");
        RightTilter = hardwareMap.get(Servo.class, "RightTilter");
        //defines Ball Magazine Servos
        Intake = hardwareMap.get(CRServo.class, "intake");
        Belt = hardwareMap.get(CRServo.class, "belt");
        LiftServo = hardwareMap.get(Servo.class, "LiftServo");

        //reverses necessary DriveBase motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //reverses necessary Shooter motor and servo
        Lshooter.setDirection(DcMotor.Direction.REVERSE);
        RightTilter.setDirection(Servo.Direction.REVERSE);

        //tells DriveBase motors to run using encoder to me more accurate
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tells Shooter motors to run using encoder to me more accurate
        Rshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        //Auto starts here:

        Wait(500);

        DriveForward(0.5, 250);

        TurnRight(0.25, 100);

        ShootBall(0.5);

        ShootBall(0.5);

        ShootBall(0.5);

        TurnLeft(0.25, 100);

    }
}