package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Main Auto Code", group = "robot")
public class MainAutoCode extends LinearOpMode{

    //declare DriveBase motors
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive;

    //Declare Ball Magazine Servos
    Servo LiftServo;
    CRServo Intake, Belt;

    //declare Shooter motors and servos
    DcMotor RightShooterMotor ,LeftShooterMotor;

    //define DriveBase functions
    public void DriveForward(double power, long duration){
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
    public void ShootBall(double power){
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
    public void SpinIntake(double power){
        Intake.setPower(power);
        Belt.setPower(power);
    }
    public void StopIntake(){
        Intake.setPower(0);
        Belt.setPower(0);
    }

    //functions to make code look better
    public void Wait(long duration){
        //just to make code more readable
        sleep(duration);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //define DriveBase motors
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        //defines Shooter motors and servos
        RightShooterMotor = hardwareMap.get(DcMotor.class, "Right_shooter_motor");
        RightShooterMotor = hardwareMap.get(DcMotor.class, "Left_shooter_motor");

        //defines Ball Magazine Servos
        Intake = hardwareMap.get(CRServo.class, "intake");
        Belt = hardwareMap.get(CRServo.class, "belt");
        LiftServo = hardwareMap.get(Servo.class, "LiftServo");

        //reverses necessary DriveBase motors
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //reverses necessary Shooter motor
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

        Wait(500);

        DriveForward(0.5, 250);

        TurnRight(0.25, 100);

        ShootBall(0.5);

        ShootBall(0.5);

        ShootBall(0.5);

        TurnLeft(0.25, 100);

        DriveForward(0.5, 250);

    }
}