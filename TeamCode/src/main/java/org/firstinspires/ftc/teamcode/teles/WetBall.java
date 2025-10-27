package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.autos.BlueAuto;
import org.firstinspires.ftc.teamcode.config.BBG;
import org.firstinspires.ftc.teamcode.config.GreenFN;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.control.PIDFController;



import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class WetBall extends OpMode {


    GreenFN g;
    BBG gp1, gp2;
    boolean autoDriving = false;
    boolean headingLock = false;
    double headingError = 0;
    double lastHeading;
    List<Action> runningActions = new ArrayList<>();
    Pose startPose = new Pose(0,0,Math.toRadians(135));
    Pose launchPose = new Pose(), farPose = new Pose();
    Pose intakePose = new Pose();
    public static PIDFController headingPIDF;
    Timer loopTimer;
    public static double headingLockTolerance = 0.007;
    public static double hp = 1, hi = 0, hd=0.08, hf=0.01;
    double lastLoopTime;
    @Override
    public void init() {
        g = new GreenFN(hardwareMap);
        g.follower.setStartingPose(BlueAuto.startPose);
        lastHeading = BlueAuto.startPose.getHeading();
        g.follower.startTeleopDrive();

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);
        headingPIDF = new PIDFController(new PIDFCoefficients(hp, hi, hd, hf));
        loopTimer = new Timer();
        lastLoopTime = loopTimer.getElapsedTimeSeconds();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        g.follower.startTeleopDrive();
        GreenFN.intakeTarget = GreenFN.intakeRPM;
        GreenFN.shooterTarget = GreenFN.shooterCloseRPM;
    }

    @Override
    public void loop() {
        g.update();
        double sprint = 1-(Math.abs(gp1.left_trigger) * 0.8);

        if(gp1.dpad_down())  {
            g.follower.setPose(g.getPose().copy().setHeading(0));
            lastHeading = 0;
        }

        if(gp2.a()  || gp1.right_trigger()) {g.shoot(4);}
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
                GreenFN.shooterTarget = GreenFN.shooterCloseRPM;
        }

        if (gp2.dpad_up()) {
            GreenFN.shooterTarget = GreenFN.shooterFarRPM;
        }

        if (gp2.dpad_down()) {
            g.stopper1.setPosition(GreenFN.stopperShoot);
            g.stopper2.setPosition(GreenFN.stopperShoot);
            g.stopper3.setPosition(GreenFN.stopperShoot);
        }



        if(gp1.a()) {
            launchPose = g.getPose();
        }
        
        if(gp1.x()) {
            intakePose = g.getPose();
        }

        if (gp1.b()) {
            farPose = g.getPose();
        }

        if(gp1.y()) {
            Path intakePath = new Path(
                    new BezierLine(
                            g.getPose(),
                            intakePose
                    )
            );
            intakePath.setLinearHeadingInterpolation(g.getPose().getHeading(), intakePose.getHeading());
            runningActions.add(
                    new SequentialAction(
                            new InstantAction(() -> g.intakeStoppers()),
                            g.followUntilDone(
                                    intakePath
                            )
                    )
            );
        }



        if(gp1.right_bumper()) {
            GreenFN.shooterTarget = GreenFN.shooterFarRPM;
            Path farZonePath = new Path(
                    new BezierLine(
                            g.getPose(),
                            farPose
                    )
            );
            farZonePath.setLinearHeadingInterpolation(g.getPose().getHeading(), farPose.getHeading());
            runningActions.add(
                    new SequentialAction(
                            g.loadAction(),
                            g.followUntilDone(
                                    farZonePath
                            )
                    )
            );
        }

        if(gp1.left_bumper()) {
            GreenFN.shooterTarget = GreenFN.shooterCloseRPM;
            Path drivePath = new Path(
                    new BezierLine(
                            g.getPose(),
                            launchPose
                    )
            );
            drivePath.setLinearHeadingInterpolation(g.getPose().getHeading(), launchPose.getHeading());
            runningActions.add(
                    new SequentialAction(
                        g.loadAction(),
                        g.followUntilDone(
                                drivePath,
                                true
                        )
                    )
            );
        }

        if(gp1.ps || gp1.touchpad) {
            runningActions = new ArrayList<>();

        }


        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        double headingPower = 0;


        autoDriving = g.follower.isBusy() || autoDriving;


        if ((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) != 0 || !autoDriving) {
            if (autoDriving) {
                autoDriving = false;
                g.follower.breakFollowing();
                g.follower.startTeleopDrive();
            }
            if(Math.abs(gamepad1.right_stick_x) > 0.01) {
                headingPower = -gamepad1.right_stick_x * 0.75 * sprint;
                sprint = 1;
                headingLock = false; // temporarily release heading lock while turning
            }
            else if(!headingLock && Math.abs(g.angvel) < headingLockTolerance) {
                lastHeading = g.getPose().getHeading(); // only set lastHeading once heading lock re-engages
                headingLock = true;
            }
            if(headingLock) {
                headingError = MathFunctions.getTurnDirection(g.getPose().getHeading(), lastHeading) *
                        MathFunctions.getSmallestAngleDifference(g.getPose().getHeading(), lastHeading);
                headingPIDF.updateFeedForwardInput(MathFunctions.getTurnDirection(g.getPose().getHeading(), lastHeading));
                headingPIDF.updateError(headingError);
                headingPower = headingPIDF.run();
            }

            g.follower.setTeleOpDrive(-gamepad1.left_stick_y  * sprint, -gamepad1.left_stick_x * sprint * 1.27, headingPower, false);
        }


        runningActions = newActions;



        telemetry.addData("Current Motif: ", GreenFN.currentMotif);
        telemetry.addData("intake velocity: ", g.iv);
        telemetry.addData("intake target: ", GreenFN.intakeTarget);
        telemetry.addData("intake power: ", GreenFN.intakePower);

        telemetry.addData("shooter1 velo: ", g.s1v);
        telemetry.addData("shooter2 velo: ", g.s2v);
        telemetry.addData("shooter target: ", GreenFN.shooterTarget);
        telemetry.addData("sh1 pow: ", GreenFN.flywheel1Power);
        telemetry.addData("sh2 pow: ", GreenFN.flywheel2Power);
        telemetry.addData("Autodriving: ", autoDriving);
        telemetry.addData("Sprint: ", sprint);
        telemetry.addData("X: ", g.getPose().getX());
        telemetry.addData("Y", g.getPose().getY());
        telemetry.addData("heading: ", g.getPose().getHeading());
        telemetry.addData("heading power: ", headingPower);
        telemetry.addData("angulra velo: ", g.angvel);
        telemetry.addData("heading lock: ", headingLock);
        telemetry.addData("heading error: ", headingError);
        telemetry.addData("apriltagx", GreenFN.fromApriltag.getX());
        telemetry.addData("apriltagy", GreenFN.fromApriltag.getY());
        telemetry.addData("apriltagheading", GreenFN.fromApriltag.getHeading());
        telemetry.addData("looptime: ", loopTimer.getElapsedTimeSeconds()-lastLoopTime);
        lastLoopTime = loopTimer.getElapsedTimeSeconds();

        telemetry.update();
    }
}
