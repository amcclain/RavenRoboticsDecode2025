package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

/**
 * Blue team far starting position autonomous.
 * Uses far encoder constants (41 counts/inch, 10.5 counts/degree).
 * Mirror of RedFarAuto with left/right directions swapped.
 */
@Autonomous(name = "Blue Far Auto", group = "robot")
public class BlueFarAuto extends BaseAutonomous {

    @Override
    protected double getCountsPerInch() {
        return RobotConstants.FAR_COUNTS_PER_INCH;
    }

    @Override
    protected double getCountsPerDegree() {
        return RobotConstants.FAR_COUNTS_PER_DEGREE;
    }

    @Override
    protected boolean isRedTeam() {
        return false;
    }

    @Override
    protected void runAutonomousRoutine() throws InterruptedException {
        // Drive forward
        driveDistance("forward", 0.5, 6);
        Wait(1000);

        // Turn to face tower (mirrored from red)
        turnDegrees("left", 0.5, 22);
        Wait(1000);

        // Shoot balls
        unloadBalls(0.61);

        // Drive forward to park
        driveDistance("forward", 0.5, 12);
        Wait(1000);
    }
}
