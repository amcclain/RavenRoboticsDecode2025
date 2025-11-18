package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "New Auto", group = "robot")
public class New_Auto extends LinearOpMode {
    DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, rightShooterMotor, leftShooterMotor;
    DcMotor intake, belt;
    Servo ballLifter;

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
        int frontLeftTarget = frontLeftDrive.getCurrentPosition()+rotations;
        int frontRightTarget = frontLeftDrive.getCurrentPosition()+rotations;
        int backLeftTarget = frontLeftDrive.getCurrentPosition()+rotations;
        int backRightTarget = frontLeftDrive.getCurrentPosition()+rotations;

        frontLeftDrive.setTargetPosition(frontLeftTarget);
        frontRightDrive.setTargetPosition(frontRightTarget);
        backLeftDrive.setTargetPosition(backLeftTarget);
        backRightDrive.setTargetPosition(backRightTarget);

        while ( frontLeftDrive.isBusy() ||
                frontRightDrive.isBusy() ||
                backLeftDrive.isBusy() ||
                backRightDrive.isBusy()
        ){
            sleep(5);
        }
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


        waitForStart();
//--------------------------------------------------------------------------------------------------
//      Auto starts
//--------------------------------------------------------------------------------------------------

        newDriveForward(10);

    }

}
