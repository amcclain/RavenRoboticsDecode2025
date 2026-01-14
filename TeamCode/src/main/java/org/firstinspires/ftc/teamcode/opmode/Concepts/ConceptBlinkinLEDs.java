package org.firstinspires.ftc.teamcode.opmode.Concepts;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.WaverlyGamepad;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "LEDs", group = "Robot")
public class ConceptBlinkinLEDs extends OpMode {

    RevBlinkinLedDriver led;
    WaverlyGamepad gp = null;
    List<RevBlinkinLedDriver.BlinkinPattern> list = new ArrayList<>();

    @Override
    public void init() {
        led = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        gp = new WaverlyGamepad(gamepad1);

        list.add(RevBlinkinLedDriver.BlinkinPattern.RED);
        list.add(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        list.add(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        list.add(RevBlinkinLedDriver.BlinkinPattern.LIME);
        list.add(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        list.add(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
        list.add(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        list.add(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
        list.add(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        list.add(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }


    @Override
    public void loop() {
        gp.readButtons();


    }
}
