package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Showcase", group = "Robot")
public class Show_Case extends OpMode {

    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, intake, belt;
    DcMotorEx rightShooterMotor, leftShooterMotor;
    Servo liftServo;
    WaverlyGamepad gp = null;
    private AprilTagProcessor aprilTag;
    IMU imu;

    int modeNum = 1;
    int loopNum = 0;
    boolean driving = false;

    @Override
    public void init() {

        defineMotors();

        initCamera();

        initIMU();

        gp = new WaverlyGamepad(gamepad1);

        //init loop
        while (true){

            //use bumpers to adjust modeNum
            if (gp.leftBumperPressed) {
                modeNum--;
                if (modeNum < 1) modeNum = 3;
            } else if (gp.rightBumperPressed) {
                modeNum++;
                if (modeNum > 3) modeNum = 1;
            }

            //select mode

            //output to the screen
            telemetry.addLine("Select mode:");
            telemetry.addLine("");
            telemetry.addLine((modeNum == 1? "> " : "") + "Back to Start mode");
            telemetry.addLine((modeNum == 2? "> " : "") + "Shoot Tag mode");
            telemetry.addLine((modeNum == 3? "> " : "") + "field Position mode");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("Button pressed on gamepad1:");
            telemetry.addLine("  left bumper " + gp.leftBumper);
            telemetry.addLine("  right bumper " + gp.rightBumper);

            telemetry.update();
        }
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void loop() {
        switch (modeNum){
            case 1:
                mode1();
                break;
            case 2:
                ///mode2();
                break;
            case 3:
                ///mode3();
                break;
        }
    }

    void mode1(){
        if (loopNum == 1) {
            frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(STOP_AND_RESET_ENCODER);
        }

        if (loopNum == 20){
            driving = true;
        }

        if (gp.aPressed) {
            driving = !driving;
            if (driving){
                frontLeftDrive.setMode(RUN_USING_ENCODER);
                frontRightDrive.setMode(RUN_USING_ENCODER);
                backLeftDrive.setMode(RUN_USING_ENCODER);
                backRightDrive.setMode(RUN_USING_ENCODER);
            } else {
                frontLeftDrive.setMode(RUN_TO_POSITION);
                frontRightDrive.setMode(RUN_TO_POSITION);
                backLeftDrive.setMode(RUN_TO_POSITION);
                backRightDrive.setMode(RUN_TO_POSITION);

                frontLeftDrive.setTargetPosition(0);
                frontRightDrive.setTargetPosition(0);
                backLeftDrive.setTargetPosition(0);
                backRightDrive.setTargetPosition(0);

                frontLeftDrive.setPower(0.5);
                frontRightDrive.setPower(0.5);
                backLeftDrive.setPower(0.5);
                backRightDrive.setPower(0.5);
            }
        }

        if (driving) {
            drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }


    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

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

        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);
        rightShooterMotor.setMode(STOP_AND_RESET_ENCODER);
        leftShooterMotor.setMode(STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);
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
