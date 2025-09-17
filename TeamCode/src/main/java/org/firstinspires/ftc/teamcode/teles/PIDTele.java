package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.GreenFN;

@TeleOp
@Config
public class PIDTele extends OpMode {
    GreenFN g;
    @Override
    public void init() {
        g = new GreenFN(hardwareMap);
    }

    @Override
    public void loop() {
        g.update();


        telemetry.addData("intake velocity: ", g.intake.getVelocity());
        telemetry.addData("intake power: ", g.intakePower);
        telemetry.update();
    }
}
