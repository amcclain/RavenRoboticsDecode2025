package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Ball Shooter Test", group="Test OpModes")
public class BallShooterTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0;
    private DcMotor motor1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

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
            boolean XPressed = gamepad1.a;
            telemetry.addData("Button X", XPressed);
            if (XPressed){
                motor0.setPower(1);
                motor1.setPower(-1);
            } else {
                motor0.setPower(0.0);
                motor1.setPower(0.0);
            }

            telemetry.update();
        }
    }
}
