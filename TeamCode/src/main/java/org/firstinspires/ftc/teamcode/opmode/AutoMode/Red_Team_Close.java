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
    DcMotor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive;

    //Declare Ball Magazine Servos
    Servo LiftServo;
    CRServo Intake, Belt;

    //declare Shooter motors and servos
    DcMotor RightShooterMotor ,LeftShooterMotor;

    WaverlyGamepad wgp;

    long AdjustableTime = 2700;
    double AdjustablePower = 0.4;

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
        Intake.setPower(1);
        sleep(1250);
        LiftServo.setPosition(0.3);
        sleep(1000);
        Intake.setPower(0);
        LiftServo.setPosition(0);
        RightShooterMotor.setPower(0);
        LeftShooterMotor.setPower(0);
        sleep(1250);
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
        FrontLeftDrive.setDirection(REVERSE);
        RightShooterMotor.setDirection(REVERSE);


        //tells DriveBase motors to run using encoder to me more accurate
        FrontLeftDrive.setMode(RUN_USING_ENCODER);
        FrontRightDrive.setMode(RUN_USING_ENCODER);
        BackLeftDrive.setMode(RUN_USING_ENCODER);
        BackRightDrive.setMode(RUN_USING_ENCODER);

        //tells Shooter motors to run using encoder to me more accurate
        RightShooterMotor.setMode(RUN_USING_ENCODER);
        LeftShooterMotor.setMode(RUN_USING_ENCODER);

        if (wgp == null){
            wgp = new WaverlyGamepad(gamepad1);
        }

        while (!isStarted()){
            wgp.readButtons();

            //change time
            if (wgp.dpadDownPressed) {
                AdjustableTime = Math.max(AdjustableTime - 100, 0);
            } else if (wgp.dpadUpPressed){
                AdjustableTime += 100;
            }

            //change power
            if (wgp.dpadLeftPressed) {
                AdjustablePower = Math.max(AdjustablePower - 0.05, 0);
            } else if (wgp.dpadRightPressed){
                AdjustablePower += 0.05;
            }

            telemetry.addLine("driving time is: " + AdjustableTime);
            telemetry.addLine("shoot power is: " + AdjustablePower);
            telemetry.update();
        }


        waitForStart();
//--------------------------------------------------------------------------------------------------
//      Auto starts
//--------------------------------------------------------------------------------------------------

        Belt.setPower(1);

        Wait(500);

        DriveBackward(0.5, 2700);

        ShootBall(0.4);

        ShootBall(0.4);

        ShootBall(0.4);





    }
}