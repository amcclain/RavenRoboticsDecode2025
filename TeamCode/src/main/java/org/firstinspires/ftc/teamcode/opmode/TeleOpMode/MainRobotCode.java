/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.opmode.TeleOpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;


@TeleOp(name = "Main", group = "Robot")
public class MainRobotCode extends OpMode {

    //declares the motors and servos
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotorEx Rshooter ,Lshooter;
    CRServo belt, intake;
    Servo BallLift;

    WaverlyGamepad gp = null;

    //declares the Inertial Measurement Unit
    IMU imu;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightMotor");
        Rshooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        Lshooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
        intake = hardwareMap.get(CRServo.class, "Intake");
        belt = hardwareMap.get(CRServo.class, "Belt");
        BallLift = hardwareMap.get(Servo.class, "BallLift");

        //flips the direction of the necessary motors and servos
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        Rshooter.setDirection(DcMotor.Direction.REVERSE);

        //tells motors to use RUN_USING_ENCODER to be more accurate
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gp = new WaverlyGamepad(gamepad1);

        //setting up the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    int velocity = 1000;
    int maxVelocity = 2000;
    int stepVelocity = 20*1;
    int direction = 1;
    boolean shooting = false;
    boolean relativeDrive = true, intakeActive = false;

    @Override
    public void loop() {
        //add telemetry
        telemetry.addLine("Press Down D-Pad to reset Yaw");
        telemetry.addLine("Press Up D-Pad to toggle between robot and field relative");
        telemetry.addLine("The left joystick moves robot");
        telemetry.addLine("The right joystick turns the robot");
        telemetry.addLine("Shooter power: " + velocity/20d + "%");
        telemetry.addLine("Current shooter velocity Left: " + Lshooter.getVelocity() + " Right: " + Rshooter.getVelocity());
        telemetry.addLine("Intake Status: " + (direction == 1? "Intake" : "Eject"));
        telemetry.addLine("Intake Active: " + intakeActive);


        gp.readButtons();


        //reset robot Yaw
        if (gp.dpadDownPressed) {
            imu.resetYaw();
        }


        //intake
        if (gp.aPressed){
            intakeActive = !intakeActive;
        }
        if (gp.rightTriggerPressed){
            direction = -direction;
        }
        if (intakeActive) {
            intake.setPower(direction);
            /*if (gp.b)
                belt.setPower(0);
            else*/
                belt.setPower(direction);
        } else {
            intake.setPower(0);
            belt.setPower(0);
        }


        //shooting
        if (gp.leftBumperPressed){
            velocity = Math.max(velocity - stepVelocity, 0);
        }
        if (gp.rightBumperPressed){
            velocity = Math.min(velocity + stepVelocity, maxVelocity);
        }
        if (gp.yPressed){
            shooting = !shooting;
        }
        if (shooting){
            Rshooter.setVelocity(velocity);
            Lshooter.setVelocity(velocity);
        } else {
            Rshooter.setVelocity(0);
            Lshooter.setVelocity(0);
        }


        //Lift servo
        if (gp.b){
            BallLift.setPosition(0.3);
        } else {
            BallLift.setPosition(0);
        }


        //driving
        if (gp.dpadUpPressed){
            relativeDrive = !relativeDrive;
        }
        if (relativeDrive) {
            drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


    }


    //Field relative drive
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    //traditional drive
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


}
