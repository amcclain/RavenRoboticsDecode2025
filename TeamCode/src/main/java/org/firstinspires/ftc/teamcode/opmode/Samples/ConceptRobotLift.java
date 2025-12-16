package org.firstinspires.ftc.teamcode.opmode.Samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Red Main", group = "Robot")
public class ConceptRobotLift extends OpMode {
    Servo leftServo, rightServo;
    WaverlyGamepad gp = null;


    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "LeftServo");
        rightServo = hardwareMap.get(Servo.class, "RightServo");

        gp = new WaverlyGamepad(gamepad1);
    }

    boolean lifting = false;

    @Override
    public void loop() {

        gp.readButtons();

        if (gp.aPressed){
            lifting = !lifting;
        }

        if (lifting){
            leftServo.setPosition(0.05);
            rightServo.setPosition(0.05);
        } else {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
        }

        telemetry.addLine("lifting: " + lifting);
        telemetry.addLine("a pressed: " + gp.a);

        telemetry.update();

    }

}
