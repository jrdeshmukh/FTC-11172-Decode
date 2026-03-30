package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.util.Timer;
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
    Timer recoveryTimer;
    int rpmtol = 150;
    double recoveryStarttime = 0;
    double recoveryTime = 0;
    int rpmbacktol = 30;
    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        recoveryTimer = new Timer();
        GreenFN.inAuto = true;
        g.loaded = true;
    }

    @Override
    public void loop() {
        g.update();

        if(Math.abs(g.s1v - GreenFN.shooterTarget)>rpmtol) {
            recoveryStarttime = recoveryTimer.getElapsedTimeSeconds();
        }
        if(Math.abs(g.s1v-GreenFN.shooterTarget)<rpmbacktol) {
            recoveryTime = recoveryTimer.getElapsedTimeSeconds()-recoveryStarttime;
        }



        telemetry.addData("intake velocity: ", g.intake.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intake target: ", g.intakeTarget);
        telemetry.addData("intake power: ", g.intakePower);

        telemetry.addData("shooter1 velo: ", g.shooter1.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shooter2 velo: ", g.shooter2.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shooter target: ", g.shooterTarget);
        telemetry.addData("sh1 pow: ", g.flywheel1Power);
        telemetry.addData("sh2 pow: ", g.flywheel2Power);
        telemetry.addData("recovery time: ", recoveryTime);
        telemetry.update();
    }
}
