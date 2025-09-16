package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="One Motor One Servo", group="Test OpModes")
public class OneMotorOneServoTestMode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorTest;
    private Servo servoTest;

    @Override
    public void runOpMode() throws InterruptedException {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Run Time", runtime.toString());

            // Read the joysticks - do not do anything with them, just show their values on telemetry
            double left_stick_x = gamepad1.left_stick_x;
            double left_stick_y = gamepad1.left_stick_y;
            double right_stick_x = gamepad1.right_stick_x;
            double right_stick_y = gamepad1.right_stick_y;
            telemetry.addData("Joystick", "lX (%.2f), lY (%.2f), rX (%.2f), rY (%.2f)", left_stick_x, left_stick_y, right_stick_x, right_stick_y);

            // Read if button A is pressed - if so, turn motor on at half power, else turn it off
            boolean aPressed = gamepad1.a;
            telemetry.addData("Button A", aPressed);

            if (aPressed) {
                motorTest.setPower(0.5);
            } else {
                motorTest.setPower(0.0);
            }

            // Read if button B is pressed - if so, set servo to position 1, else set it to position 0
            boolean bPressed = gamepad1.b;
            telemetry.addData("Button B", bPressed);
            if (bPressed) {
                servoTest.setPosition(1.0);
            } else {
                servoTest.setPosition(0.0);
            }

            telemetry.update();
        }
    }
}
