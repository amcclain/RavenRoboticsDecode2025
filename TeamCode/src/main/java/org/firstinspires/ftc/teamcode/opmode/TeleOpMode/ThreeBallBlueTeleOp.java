package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

/**
 * TeleOp mode with CRServo lift mechanism for the 3-ball robot variant (Blue team).
 * Uses DPad left/right to control the lift servos.
 */
@TeleOp(name = "3 Ball Blue TeleOp", group = "Robot")
public class ThreeBallBlueTeleOp extends BaseTeleOp {

    // CRServos for lift mechanism
    private CRServo leftLift;
    private CRServo rightLift;
    private CRServo middleLift;

    @Override
    protected boolean isRedTeam() {
        return false;  // Blue team
    }

    @Override
    protected void initExtraHardware() {
        // Initialize CRServos
        hardware.initCRServos(hardwareMap);
        leftLift = hardware.leftLift;
        rightLift = hardware.rightLift;
        middleLift = hardware.middleLift;
    }

    @Override
    protected void extraLoop() {
        // DPad left/right controls the lift servos
        if (gp.dpadLeft) {
            leftLift.setPower(1);
            rightLift.setPower(1);
            middleLift.setPower(1);
        } else if (gp.dpadRight) {
            leftLift.setPower(-1);
            rightLift.setPower(-1);
            middleLift.setPower(-1);
        } else {
            leftLift.setPower(0);
            rightLift.setPower(0);
            middleLift.setPower(0);
        }

        // Additional telemetry for lift
        telemetry.addLine("");
        telemetry.addLine("DPad Left/Right controls lift");
    }
}
