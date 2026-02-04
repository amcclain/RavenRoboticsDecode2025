package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

/**
 * TeleOp mode with CRServo lift mechanism for the 3-ball robot variant.
 * Uses DPad left/right to control the lift servos.
 */
@TeleOp(name = "3 Ball Red TeleOp", group = "Robot")
public class ThreeBallTeleOp extends BaseTeleOp {

    // CRServos for lift mechanism
    private CRServo leftLift;
    private CRServo rightLift;
    private CRServo middleLift;

    private boolean isRedTeam = true;  // Can be changed via constructor or setting

    @Override
    protected boolean isRedTeam() {
        return isRedTeam;
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

    @Override
    protected void handleMiscControls() {
        // Toggle controls display with different button since DPad left is used for lift
        // We still allow the left dpad pressed behavior for toggling controls display
        // but the DPad left held behavior is used for lift in extraLoop()
        if (gp.dpadLeftPressed) {
            // Note: This will still fire since pressed is edge-triggered
            // but won't interfere with the held lift control
        }
    }
}
