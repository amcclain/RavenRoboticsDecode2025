package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;


@Autonomous(name = "Ravens Autonomous", group = "robot")
@Disabled
public class RavensAutonomous extends OpMode {

    // Settings
    double COUNTS_PER_INCH = 29.8;
    int targetInches = 96;
    double power = 0.5;
    String dir = "back";

    // Hardware
    WaverlyGamepad gp1;

    // Wheels
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;


    public void init() {
        // Initialize hardware
        gp1 = new WaverlyGamepad(gamepad1);


        fl = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        fr = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        bl = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        br = hardwareMap.get(DcMotor.class, "BackRightMotor");

        bl.setDirection(REVERSE);
        fl.setDirection(REVERSE);
        fr.setDirection(REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void init_loop() {
        gp1.readButtons();

        //direction
        if (gp1.dpadLeftPressed) {
            dir = "left";
        } else if (gp1.dpadRightPressed) {
            dir = "right";
        } else if (gp1.dpadUpPressed) {
            dir = "forward";
        } else if (gp1.dpadDownPressed) {
            dir = "back";
        }

        //power change
        if (gp1.leftTriggerPressed) {
            power = Math.max(power - 0.1, 0.1);
        } else if (gp1.rightTriggerPressed) {
            power = Math.min(power + 0.1, 1.0);
        }


        if (gp1.leftBumperPressed) {
            targetInches = Math.max(targetInches - 1, 1);
        } else if (gp1.rightBumperPressed) {
            targetInches++;
        }

        if (gp1.yPressed) {
            COUNTS_PER_INCH++;
        } else if (gp1.aPressed) {
            COUNTS_PER_INCH--;
        }

        if (gp1.xPressed) {
            COUNTS_PER_INCH += 0.1;
        } else if (gp1.bPressed) {
            COUNTS_PER_INCH -= 0.1;
        }

        telemetry.addData("targetInches", targetInches);
        telemetry.addData("dir", dir);
        telemetry.addData("power", power);
        telemetry.addData("counts per inch", COUNTS_PER_INCH);
        telemetry.addLine("");
        telemetry.addData("fl", fl.getCurrentPosition());
        telemetry.addData("bl", bl.getCurrentPosition());
        telemetry.addData("fr", fr.getCurrentPosition());
        telemetry.addData("br", br.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {

        // Drive forward
        int targetPosition = (int) (targetInches * COUNTS_PER_INCH);
        driveToPosition(targetPosition);
    }


    private void driveToPosition(int targetPosition) {
        if (dir.equals("left")|| dir.equals("right")){
            targetPosition= (int) Math.round(targetPosition*1.05);
        }
        int flPos = targetPosition;
        int blPos = targetPosition;
        int frPos = targetPosition;
        int brPos = targetPosition;

        if (dir.equals("forward")) {
            flPos = flPos * -1;
            blPos = blPos * -1;
            frPos = frPos * -1;
            brPos = brPos * -1;
        } else if (dir.equals("right")) {
            flPos = flPos * -1;
            brPos = brPos * -1;
        } else if (dir.equals("left")) {
            frPos = frPos * -1;
            blPos = blPos * - 1;
        }

        br.setTargetPosition(brPos);
        bl.setTargetPosition(blPos);
        fl.setTargetPosition(flPos);
        fr.setTargetPosition(frPos);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setPower(power);
        bl.setPower(power);
        fl.setPower(power);
        fr.setPower(power);
        telemetry.addData("fl", fl.getCurrentPosition());
        telemetry.addData("bl", bl.getCurrentPosition());
        telemetry.addData("fr", fr.getCurrentPosition());
        telemetry.addData("br", br.getCurrentPosition());
        telemetry.update();
    }

}