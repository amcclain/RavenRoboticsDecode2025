package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Team Far (old)", group = "robot")
public class Red_Team_Far_Old extends LinearOpMode{

    //declare DriveBase motors
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive;

    //Declare Ball Magazine Servos
    Servo LiftServo;
    CRServo Intake, Belt;

    //declare Shooter motors and servos
    DcMotor RightShooterMotor ,LeftShooterMotor;

    /*define DriveBase functions
    public void DriveForward(double power, long duration){
        `FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);`
    }
    public void DriveBackward(double power, long duration){
        FrontLeftDrive.setPower(-power);
        FrontRightDrive.setPower(-power);
        BackLeftDrive.setPower(-power);
        BackRightDrive.setPower(-power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
    }
    public void TurnRight(double power, long duration){
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(-power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(-power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
    }
    public void TurnLeft(double power, long duration){
        FrontLeftDrive.setPower(-power);
        FrontRightDrive.setPower(power);
        BackLeftDrive.setPower(-power);
        BackRightDrive.setPower(power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
    }

    //define Shooter and Ball Magazine functions
    public void ShootBall(double power) {
        RightShooterMotor.setPower(power);
        LeftShooterMotor.setPower(power);
        Belt.setPower(1);
        sleep(250);
        LiftServo.setPosition(0.2);
        Belt.setPower(0);
        sleep(100);
        LiftServo.setPosition(0);
        RightShooterMotor.setPower(0);
        LeftShooterMotor.setPower(0);
        sleep(100);
    }
    */

    double power = 0;
    long duration = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //define DriveBase motors
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRightMotor");

        //defines Shooter motors
        RightShooterMotor = hardwareMap.get(DcMotor.class, "RightShooterMotor");
        LeftShooterMotor = hardwareMap.get(DcMotor.class, "LeftShooterMotor");

        //defines Ball Magazine Servos
        Intake = hardwareMap.get(CRServo.class, "Intake");
        Belt = hardwareMap.get(CRServo.class, "Belt");
        LiftServo = hardwareMap.get(Servo.class, "BallLift");


        //reverses motors that are on backwards
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftShooterMotor.setDirection(DcMotor.Direction.REVERSE);


        //tells DriveBase motors to run using encoder to me more accurate
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //tells Shooter motors to run using encoder to me more accurate
        RightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
//--------------------------------------------------------------------------------------------------
//      Auto starts
//--------------------------------------------------------------------------------------------------



        power = 0.5; duration = 250;
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);

        power = 0.25; duration = 100;
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(-power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(-power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);

        power = 0.5;
        RightShooterMotor.setPower(power);
        LeftShooterMotor.setPower(power);
        Belt.setPower(1);
        sleep(250);
        LiftServo.setPosition(0.2);
        Belt.setPower(0);
        sleep(100);
        LiftServo.setPosition(0);
        RightShooterMotor.setPower(0);
        LeftShooterMotor.setPower(0);
        sleep(100);

        power = 0.5;
        RightShooterMotor.setPower(power);
        LeftShooterMotor.setPower(power);
        Belt.setPower(1);
        sleep(250);
        LiftServo.setPosition(0.2);
        Belt.setPower(0);
        sleep(100);
        LiftServo.setPosition(0);
        RightShooterMotor.setPower(0);
        LeftShooterMotor.setPower(0);
        sleep(100);

        power = 0.5;
        RightShooterMotor.setPower(power);
        LeftShooterMotor.setPower(power);
        Belt.setPower(1);
        sleep(250);
        LiftServo.setPosition(0.2);
        Belt.setPower(0);
        sleep(100);
        LiftServo.setPosition(0);
        RightShooterMotor.setPower(0);
        LeftShooterMotor.setPower(0);
        sleep(100);

        power = 0.25; duration = 100;
        FrontLeftDrive.setPower(-power);
        FrontRightDrive.setPower(power);
        BackLeftDrive.setPower(-power);
        BackRightDrive.setPower(power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);

        power = 0.5; duration = 250;
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(power);
        sleep(duration);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);

    }
}