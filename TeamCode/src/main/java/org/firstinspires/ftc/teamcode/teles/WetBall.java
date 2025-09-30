package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    Pose startPose = new Pose(0,0,Math.toRadians(135));
    Pose launchPose = new Pose();
    @Override
    public void init() {
        g = new GreenFN(hardwareMap);
        g.follower.setStartingPose(new Pose(0,0,0));
        g.follower.startTeleopDrive();

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        g.follower.startTeleopDrive();
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
            g.follower.setTeleOpDrive(-gamepad1.left_stick_y  * sprint, -gamepad1.left_stick_x * sprint * 1.27, -gamepad1.right_stick_x * 0.75 * sprint, false);
        }

        if(gp1.b())  {
            g.follower.setPose(g.follower.getPose().copy().setHeading(0));
        }

        if(gp2.a()  || gp1.right_bumper()) {g.shoot(4);}
        if(gp2.x()) {g.shoot(3);}
        if(gp2.y()) {g.shoot(2);}
        if(gp2.b()) {g.shoot(1);}

        if(gp2.left_bumper()) {g.intakeStoppers();}
        if(gp2.right_bumper()) {g.loadStoppers();}
        if(gp2.left_trigger()) {
            if(GreenFN.intakeTarget == GreenFN.intakeRPM) {
                GreenFN.intakeTarget = -GreenFN.intakeRPM;
            }
            else {
                GreenFN.intakeTarget = GreenFN.intakeRPM;
            }
        }
        if(gp2.right_trigger()) {
            if(g.shooterTarget == g.shooterCloseRPM) {
                g.shooterTarget = 0;
            }
            else {
                g.shooterTarget = g.shooterCloseRPM;
            }
        }

        if(gp1.a()) {
            launchPose = g.follower.getPose();
        }

        if(gp1.left_bumper()) {
            runningActions.add(
                    new SequentialAction(
                        g.followUntilDone(
                                new Path(
                                        new BezierLine(
                                                g.follower.getPose(),
                                                launchPose
                                        )
                                )
                        ),
                        new InstantAction(() -> g.follower.holdPoint(launchPose))
                    )
            );
        }


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
        telemetry.addData("intake velocity: ", g.intake.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intake target: ", g.intakeTarget);
        telemetry.addData("intake power: ", g.intakePower);

        telemetry.addData("shooter1 velo: ", g.shooter1.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shooter2 velo: ", g.shooter2.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shooter target: ", g.shooterTarget);
        telemetry.addData("sh1 pow: ", g.flywheel1Power);
        telemetry.addData("sh2 pow: ", g.flywheel2Power);
        telemetry.addData("Autodriving: ", autoDriving);
        telemetry.addData("Sprint: ", sprint);
        telemetry.addData("X: ", g.follower.getPose().getX());
        telemetry.addData("Y", g.follower.getPose().getY());
        telemetry.addData("heading: ", g.follower.getPose().getHeading());
        telemetry.update();
    }
}
