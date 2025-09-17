package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.config.GreenFN.currentMotif;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.io.File;

@Autonomous
public class SampleAuto extends OpMode {

    GreenFN g;
    @Override
    public void init() {

        g = new GreenFN(hardwareMap);
        currentMotif = null;

    }
    @Override
    public void loop() {
        g.update();
        telemetry.addData("Current motif:", currentMotif);
        telemetry.update();
    }
}
