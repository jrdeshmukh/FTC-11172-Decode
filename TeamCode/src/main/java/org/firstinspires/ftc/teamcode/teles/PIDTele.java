package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.util.List;

@TeleOp
@Config
public class PIDTele extends OpMode {
    GreenFN g;
    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        g.update();


        telemetry.addData("intake velocity: ", g.intake.getVelocity(AngleUnit.DEGREES)/6.0);
        telemetry.addData("intake target: ", g.intakeTarget);
        telemetry.addData("intake power: ", g.intakePower);

        telemetry.addData("shooter1 velo: ", g.shooter1.getVelocity(AngleUnit.DEGREES)/6.0);
        telemetry.addData("shooter2 velo: ", g.shooter2.getVelocity(AngleUnit.DEGREES)/6.0);
        telemetry.addData("shooter target: ", g.shooterTarget);
        telemetry.addData("sh1 pow: ", g.flywheel1Power);
        telemetry.addData("sh2 pow: ", g.flywheel2Power);
        telemetry.update();
    }
}
