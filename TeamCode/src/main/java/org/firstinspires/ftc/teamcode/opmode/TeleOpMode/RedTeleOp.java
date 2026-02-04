package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp mode for the Red team.
 */
@TeleOp(name = "Red TeleOp", group = "Robot")
public class RedTeleOp extends BaseTeleOp {

    @Override
    protected boolean isRedTeam() {
        return true;
    }
}
