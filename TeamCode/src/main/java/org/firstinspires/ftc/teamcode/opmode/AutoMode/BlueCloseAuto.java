package org.firstinspires.ftc.teamcode.opmode.AutoMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

/**
 * Blue team close starting position autonomous.
 * Uses close encoder constants (29.8 counts/inch, 7.4 counts/degree).
 * Mirror of RedCloseAuto with left/right directions swapped.
 */
@Autonomous(name = "Blue Close Auto", group = "robot")
public class BlueCloseAuto extends BaseAutonomous {

    @Override
    protected double getCountsPerInch() {
        return RobotConstants.CLOSE_COUNTS_PER_INCH;
    }

    @Override
    protected double getCountsPerDegree() {
        return RobotConstants.CLOSE_COUNTS_PER_DEGREE;
    }

    @Override
    protected boolean isRedTeam() {
        return false;
    }

    @Override
    protected void runAutonomousRoutine() throws InterruptedException {
        // Pre-warm intake and belt
        hardware.intake.setPower(0.2);
        hardware.belt.setPower(0.5);
        Wait(500);

        // Drive backward to scoring position
        driveDistance("backward", 0.8, 93);
        Wait(2000);

        // Push into wall
        driveConstantPower(0.5);
        Wait(1000);

        // Shoot balls
        unloadBalls(0.49);

        // Turn left to face ball pickup (mirrored from red)
        turnDegrees("left", 0.8, 57);
        Wait(750);

        // Drive forward to pick up balls
        driveDistance("forward", 0.5, 72);
        spinIntake();
        Wait(2500);

        // Drive back to scoring position
        driveDistance("backward", 0.5, 72);
        Wait(2500);
        stopIntake();

        // Turn right to face tower (mirrored from red)
        turnDegrees("right", 0.8, 58);
        Wait(750);

        // Prepare and shoot second round
        hardware.intake.setPower(0.2);
        unloadBalls(0.49);

        // Park (mirrored from red)
        driveDistance("left", 0.8, 36);
        Wait(2000);
    }
}
