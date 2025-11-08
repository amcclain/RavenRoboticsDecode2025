package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;

@Autonomous(name = "Red Team Close", group = "robot")
public class Red_Team_Close extends LinearOpMode{

    //declare DriveBase motors
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    //Declare Ball Magazine Servos
    Servo liftServo;
    CRServo intake, belt;

    //declare Shooter motors and servos
    DcMotor rightShooterMotor, leftShooterMotor;

    WaverlyGamepad wgp;

    long adjustableTime = 1300;
    double adjustablePower = 0.4;

    //define DriveBase functions
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

    //define Shooter and Ball Magazine functions
    public void shootBall(double power){
        rightShooterMotor.setPower(power);
        leftShooterMotor.setPower(power);
        intake.setPower(1);
        sleep(1250);
        liftServo.setPosition(0.3);
        sleep(1250);
        intake.setPower(0);
        liftServo.setPosition(0);
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
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightMotor");

        //defines Shooter motors
        rightShooterMotor = hardwareMap.get(DcMotor.class, "RightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotor.class, "LeftShooterMotor");

        //defines Ball Magazine Servos
        intake = hardwareMap.get(CRServo.class, "Intake");
        belt = hardwareMap.get(CRServo.class, "Belt");
        liftServo = hardwareMap.get(Servo.class, "BallLift");


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

        if (wgp == null){
            wgp = new WaverlyGamepad(gamepad1);
        }

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

        belt.setPower(1);

        Wait(500);

        driveBackward(0.5, 2700);

        shootBall(0.4);

        shootBall(0.4);

        shootBall(0.4);

        driveRight(0.5, 1300);



    }
}