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
    boolean targetHeading = false;
    boolean inZone = false;
    double autoRPMtol = 36;
    double headingError = 0;
    double lastHeading;
    List<Action> runningActions = new ArrayList<>();
    Pose startPose = new Pose(0,0,0);
    Pose launchPose = new Pose(), farPose = new Pose();
    Pose intakePose = new Pose();
    public static PIDFController headingPIDF;
    Timer loopTimer;
    public static double headingLockTolerance = 0.007;
    public static double hp = 0.8, hi = 0, hd=0.09, hf=0.01;
    double lastLoopTime;
    Pose goalPose = new Pose(0,0,0);
    enum Side {
        RED,
        BLUE
    }
    Side side = Side.BLUE;
    @Override
    public void init() {
        g = new GreenFN(hardwareMap);
        g.follower.setStartingPose(startPose);
        lastHeading = startPose.getHeading();
        g.follower.startTeleopDrive();

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);
        headingPIDF = new PIDFController(new PIDFCoefficients(hp, hi, hd, hf));
        loopTimer = new Timer();
        lastLoopTime = loopTimer.getElapsedTimeSeconds();
        GreenFN.minTimeBtwnShots = 0;
        GreenFN.intakeRPM = 700;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    @Override
    public void init_loop() {
        if(gamepad2.dpad_left) {
            side = Side.BLUE;
        }
        if(gamepad2.dpad_right) {
            side = Side.RED;
        }
    }

    @Override
    public void start() {
        g.follower.startTeleopDrive();
        GreenFN.intakeTarget = GreenFN.intakeRPM;
        GreenFN.shooterTarget = GreenFN.shooterCloseRPM;
        lastHeading = g.getPose().getHeading();
    }

    @Override
    public void loop() {
        g.update();
        double sprint = 1;

        if(gp1.dpad_down())  {
            g.follower.setPose(g.getPose().copy().setHeading(0));
            lastHeading = 0;
        }

        if(gp2.a()  || gp1.right_trigger()) {g.shoot(4);}
        if(gp2.x()) {g.shoot(1);}
        if(gp2.y()) {g.shoot(2);}

        if(gp2.b()) {g.shoot(3);}

        if(gp2.left_bumper()) {g.intakeStoppers();}
        if(gp2.right_bumper()) {g.loadStoppers();}
//        if(gp1.left_trigger==0) {
//            GreenFN.intakeTarget = 0;
//        }
//        if(gp2.left_trigger()) {
//            if(GreenFN.intakeTarget == GreenFN.intakeRPM) {
//                GreenFN.intakeTarget = -GreenFN.intakeRPM;
//            }
//            else {
//                GreenFN.intakeTarget = GreenFN.intakeRPM;
//            }
//        }

        if(gamepad1.left_trigger>0.05F) {
            GreenFN.intakeTarget=GreenFN.intakeRPM;
        }
        else {
            GreenFN.intakeTarget = 0;
        }

        if(gamepad2.left_trigger>0.05F) {
            GreenFN.intakeTarget = -GreenFN.intakeRPM;
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
            if(g.getPose().getHeading()>1.5) {
                g.follower.setPose(new Pose(96, 96, g.getPose().getHeading()));
                goalPose = new Pose(131, 134);
            }
            else{
                g.follower.setPose(new Pose(96, 48, g.getPose().getHeading()));
                goalPose = new Pose(131, 11);
            }
//            if(side==Side.RED) {

//            }
//            if(side==Side.BLUE) {
//                g.follower.setPose(new Pose(48, 96, g.getPose().getHeading()));
//                goalPose = new Pose(13, 137);
//            }
        }

        if(gp1.x()) {
            intakePose = g.getPose();
        }

        if (gp1.b()) {
            farPose = g.getPose();
        }

        if(gp1.dpad_up()) {
            Path parkPath = new Path(
                    new BezierLine(
                            g.getPose(),
                            new Pose(37.8, 37.8)
                    )
            );
            parkPath.setLinearHeadingInterpolation(g.getPose().getHeading(), 0);
            runningActions.add(
                    new SequentialAction(
                            new InstantAction(() -> g.intakeStoppers()),
                            g.followUntilDone(
                                    parkPath
                            )
                    )
            );
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



//        if(gp1.right_bumper()) {
//            GreenFN.shooterTarget = GreenFN.shooterFarRPM;
//            Path farZonePath = new Path(
//                    new BezierLine(
//                            g.getPose(),
//                            farPose
//                    )
//            );
//            farZonePath.setLinearHeadingInterpolation(g.getPose().getHeading(), farPose.getHeading());
//            runningActions.add(
//                    new SequentialAction(
//                            g.loadAction(),
//                            g.followUntilDone(
//                                    farZonePath
//                            )
//                    )
//            );
//        }

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

        if(g.getPose().distanceFrom(launchPose)<autoRPMtol && !inZone) {
            g.loadStoppers();
            inZone = true;
        }
        if(g.getPose().distanceFrom(launchPose)>autoRPMtol) {
            inZone = false;
        }


        if(autoDriving) {
            lastHeading = g.getPose().getHeading(); // only set lastHeading once heading lock re-engages
        }
        targetHeading = gamepad1.right_bumper;
        if(targetHeading) {
            lastHeading = Math.atan2(goalPose.getY()-g.getPose().getY(), goalPose.getX()-g.getPose().getX())+Math.toRadians(90);
        }

        if ((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) != 0 || !autoDriving) {
            if (autoDriving) {
                autoDriving = false;
                g.follower.breakFollowing();
                g.follower.startTeleopDrive();
            }
            if(targetHeading) {
                headingLock = true;
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

        telemetry.addData("gamepad1 left trigger: ", gamepad1.left_trigger);
        telemetry.addData("gamepad1 left_trigger(): ", gp1.left_trigger());

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
