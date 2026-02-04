package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp mode for the Blue team.
 */
@TeleOp(name = "Blue TeleOp", group = "Robot")
public class BlueTeleOp extends BaseTeleOp {

    @Override
    protected boolean isRedTeam() {
        return false;
    }
}
