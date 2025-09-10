package org.firstinspires.ftc.teamcode.teles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.GreenFN;

@TeleOp
public class SampleTele extends OpMode {
    GreenFN g;
    @Override
    public void init() {
        g = new GreenFN(hardwareMap);
    }

    @Override
    public void loop() {
        g.update();
        
    }
}
