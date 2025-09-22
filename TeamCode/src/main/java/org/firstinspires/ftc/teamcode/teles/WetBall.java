package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.BBG;
import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class WetBall extends OpMode {


    GreenFN g;
    BBG gp1, gp2;
    boolean autoDriving = false;

    List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        g = new GreenFN(hardwareMap);

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gp2);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        g.update();
        double sprint = 1-Math.abs(gp1.left_trigger) * 0.8;



        autoDriving = g.follower.isBusy() || autoDriving;

        if ((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) != 0 || !autoDriving) {
            if (autoDriving) {
                autoDriving = false;
                g.follower.startTeleopDrive();
            }
            g.follower.setTeleOpDrive(-gamepad1.left_stick_y  * sprint, -gamepad1.left_stick_x * sprint * 1.27, -gamepad1.right_stick_x * 0.75 * sprint, true);
        }

        if(gp2.a()) {g.shoot(4);}
        if(gp2.x()) {g.shoot(1);}
        if(gp2.y()) {g.shoot(2);}
        if(gp2.b()) {g.shoot(3);}

        if(gp2.left_bumper()) {g.intakeStoppers();}
        if(gp2.right_bumper()) {g.loadStoppers();}


        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }


        runningActions = newActions;



        telemetry.addData("Current Motif: ", GreenFN.currentMotif);
        telemetry.addData("Shooter RPM: ", g.shooter1.getVelocity());
        telemetry.addData("Shooter Target RPM: ", g.shooterTarget);
        telemetry.addData("Intake RPM: ", g.intake.getVelocity());
        telemetry.addData("Intake Target RPM: ", g.intakeTarget);
        telemetry.addData("Autodriving: ", autoDriving);
        telemetry.addData("Sprint: ", sprint);
        telemetry.update();
    }
}
