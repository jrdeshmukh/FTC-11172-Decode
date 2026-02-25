package org.firstinspires.ftc.teamcode.teles;

import static org.firstinspires.ftc.teamcode.config.GreenFN.Sides.BLUE;
import static org.firstinspires.ftc.teamcode.config.GreenFN.Sides.RED;

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
import org.firstinspires.ftc.teamcode.config.FirstBoolean;
import org.firstinspires.ftc.teamcode.config.GreenFN;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;


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
    boolean balls = false;
    public static int loopsPredicted = 1, loopsBack = 1;
    List<Action> runningActions = new ArrayList<>();
    Gamepad.RumbleEffect balls2 = new Gamepad.RumbleEffect.Builder().addStep(1, 1, 100).build();
    Gamepad.RumbleEffect balls3 = new Gamepad.RumbleEffect.Builder().addStep(1, 1, 200).addStep(0,0,100).addStep(1,1,200).addStep(0,0,100).addStep(1,1,200).build();
    Pose startPose = new Pose(0,0,0);
    Pose launchPose = new Pose(), farPose = new Pose();
    Pose intakePose = new Pose();
    public static PIDFController headingPIDF, secondaryHeadingPIDF;
    Timer loopTimer;
    public static double headingLockTolerance = 0.007;
    public static double hp = 0.7 , hi = 0, hd=0.09, hf=0.01;
    public static double h2p = 1.2, h2i = 0, h2d = 0.12, h2f = 0.03;
    public static double shotTime = 0.6;
    double lastLoopTime;
    Pose goalPose = new Pose(0,0,0);
    double[] xvelo = new double[20];
    double[] yvelo = new double[20];
    public static double cameraDist=5, cameraBlueOffset=3, cameraRedOffset=-3;
    int ctr = 0;
    enum Side {
        RED,
        BLUE
    }
    Side side = Side.BLUE;
    FirstBoolean ballsFull = new FirstBoolean();
    FirstBoolean balls3better = new FirstBoolean();
    AprilTagDetection goal;
    boolean isBalls3 = false;
    double headingOffset = 0;
    double newX, newY;
    public double[] insertShiftArray(double[] input, double insertVal, double insertLoc) {
        int idx = (int) insertLoc;

        // clamp index to valid range
        if (idx < 0) idx = 0;
        if (idx >= input.length) idx = input.length - 1;

        double[] result = new double[input.length];

        // copy up to insertion index
        for (int i = 0; i < idx; i++) {
            result[i] = input[i];
        }

        // insert value
        result[idx] = insertVal;

        // shift remaining elements right, dropping the last one
        for (int i = idx + 1; i < input.length; i++) {
            result[i] = input[i - 1];
        }

        return result;
    }

    public static double redGoalX = 131, redGoalY = 134, blueGoalX = 133, blueGoalY = 9.5;

    @Override
    public void init() {
        if(GreenFN.side== BLUE) {
//            g.follower.setPose(new Pose(99, 45, g.getPose().getHeading()));
            goalPose = new Pose(blueGoalX, blueGoalY);
            startPose = new Pose(99-8.58, 45-16.932, Math.toRadians(0));
//            gp1.rumble(1000);
//            gp2.rumble(1000);
        }
        else if (GreenFN.side == RED) {
//            g.follower.setPose(new Pose(99.5, 101.0, g.getPose().getHeading()));
            startPose = new Pose(99.5-8.5, 101+15.9, Math.toRadians(0));
            goalPose = new Pose(redGoalX, redGoalY);
//            gp1.rumble(1000);
//            gp2.rumble(1000);
        }
        g = new GreenFN(hardwareMap);
        g.follower.setStartingPose(startPose);
        g.follower.setPose(startPose);
        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        GreenFN.detect = false;

        if(GreenFN.side== BLUE) {
//            g.follower.setPose(new Pose(99, 45, g.getPose().getHeading()));
            goalPose = new Pose(133, 9.5);
            startPose = new Pose(99-8.58, 45-16.932, Math.toRadians(0));
            gp1.rumble(1000);
            gp2.rumble(1000);
        }
        else if (GreenFN.side == RED) {
//            g.follower.setPose(new Pose(99.5, 101.0, g.getPose().getHeading()));
            startPose = new Pose(99.5-8.5, 101+15.9, Math.toRadians(0));
            goalPose = new Pose(131, 134);
            gp1.rumble(1000);
            gp2.rumble(1000);
        }
        lastHeading = startPose.getHeading();
        g.follower.startTeleopDrive();

        GreenFN.intakeRPM += 400;


        headingPIDF = new PIDFController(new PIDFCoefficients(hp, hi, hd, hf));
        secondaryHeadingPIDF = new PIDFController(new PIDFCoefficients(h2p, h2i, h2d, h2f));
        loopTimer = new Timer();
        lastLoopTime = loopTimer.getElapsedTimeSeconds();
        GreenFN.minTimeBtwnShots = 0.2;
        GreenFN.intakeRPM = 1300;

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
//        goal = g.getVisibleGoalPose();
        double sprint = gp1.x()?0.5:1;

        if(!balls) {
            isBalls3 = false;
        }

        if(gp1.dpad_down())  {
            g.follower.setPose(g.getPose().copy().setHeading(0));
            lastHeading = 0;
        }

//blue right middle left, red left middle right
        if(gp1.right_trigger()) {
            if(GreenFN.side == BLUE) {
                g.shoot(1);
                g.shoot(2);
                g.shoot(3);
            }
            else {
                g.shoot(3);
                g.shoot(2);
                g.shoot(1);
            }
            balls= false;
        }
        if(gp2.x()) {g.shoot(1);}
        if(gp2.y()) {g.shoot(2);}

        if(gp2.b()) {g.shoot(3);}

        if(gp2.left_bumper()) {g.intakeStoppers();}
        if(gp2.right_bumper()) {
            g.loadStoppers(); balls = false;
        }
        if(gamepad2.right_bumper) {
            GreenFN.intakeTarget = GreenFN.intakeRPM;
        }
        else {
            GreenFN.intakeTarget = 0;
        }
        if(gp2.a()) {
            if(g.hoodUp) {
                g.setHoodDown();
                g.inAuto = true;
                GreenFN.shooterTarget = GreenFN.shooterFarRPM;
                GreenFN.minTimeBtwnShots = 0.75;
            }
            else {
                g.setHoodUp();
                g.inAuto = false;
            }
        }
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

        if(gamepad1.left_trigger>0.05F || gamepad2.left_trigger>0.05F || gamepad2.right_bumper) {
            GreenFN.intakeTarget=GreenFN.intakeRPM;
//            g.toUpdateSlots = true;
        }
        else {
            GreenFN.intakeTarget = 0;
//            g.toUpdateSlots = false;
        }
        if(g.toUpdateSlots) {
            ctr = 0;
            for(boolean ball: GreenFN.ballInSlot) {
                if(ball) {
                    ctr++;
                }
            }
            if(ctr>=2) {
                balls = true;
            }
            if(ctr==3) {
                isBalls3 = true;
            }
            if(balls && balls3better.betterboolean(isBalls3)) {
                gamepad1.runRumbleEffect(balls3);
                gamepad2.runRumbleEffect(balls3);
            }
            if(ballsFull.betterboolean(balls)) {
                if(ctr==2) {
                    gamepad1.runRumbleEffect(balls2);
                    gamepad2.runRumbleEffect(balls2);
                    isBalls3 = false;
                }
                else {
                    isBalls3 = true;
                }
            }
        }

        if(gamepad2.right_trigger>0.05F || gamepad1.ps) {
            GreenFN.intakeTarget = -GreenFN.intakeRPM;
            balls = false;
        }

//        if(gp2.right_trigger()) {
//            GreenFN.shooterTarget = GreenFN.shooterCloseRPM;
//        }

//        if (gp2.dpad_up()) {
//            GreenFN.shooterTarget = GreenFN.shooterFarRPM;
//        }
//
//        if (gp2.dpad_down()) {
//            g.stopper1.setPosition(0);
//            g.stopper2.setPosition(0);
//            g.stopper3.setPosition(0);
//            g.stopper1.setPosition(GreenFN.stopperShoot);
//            g.stopper2.setPosition(GreenFN.stopperShoot);
//            g.stopper3.setPosition(GreenFN.stopperShoot);
//        }

        if(gp2.dpad_down()) GreenFN.regressionOffset -= 1;
        if(gp2.dpad_up()) GreenFN.regressionOffset += 1;


        if(gp1.a()) {
            launchPose = g.getPose();
            if(g.getPose().getHeading()>1.5) {
                g.follower.setPose(new Pose(99.5, 101.0, g.getPose().getHeading()));
                goalPose = new Pose(131, 134);
            }
            else{
                g.follower.setPose(new Pose(99, 45, g.getPose().getHeading()));
                goalPose = new Pose(131, 8);
            }
//            if(side==Side.RED) {

//            }
//            if(side==Side.BLUE) {
//                g.follower.setPose(new Pose(48, 96, g.getPose().getHeading()));
//                goalPose = new Pose(13, 137);
//            }
        }

//        if(gp1.x()) {
//            intakePose = g.getPose();
//        }

        if (gp1.b()) {
            farPose = g.getPose();
        }

//        if(gp1.dpad_up()) {
//            Path parkPath = new Path(
//                    new BezierLine(
//                            g.getPose(),
//                            new Pose(37.8, 37.8)
//                    )
//            );
//            parkPath.setLinearHeadingInterpolation(g.getPose().getHeading(), 0);
//            runningActions.add(
//                    new SequentialAction(
//                            new InstantAction(() -> g.intakeStoppers()),
//                            g.followUntilDone(
//                                    parkPath
//                            )
//                    )
//            );
//        }

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

//        if(gamepad2.a) {
//            GreenFN.shooterTarget = GreenFN.getRPMreg(g.getPose().distanceFrom(goalPose));
//        }



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
//
//        if(gp1.left_bumper()) {
//            GreenFN.shooterTarget = GreenFN.shooterCloseRPM;
//            Path drivePath = new Path(
//                    new BezierLine(
//                            g.getPose(),
//                            launchPose
//                    )
//            );
//            drivePath.setTangentHeadingInterpolation();
//            runningActions.add(
//                    new SequentialAction(
//                            g.loadAction(),
//                            g.followUntilDone(
//                                    drivePath,
//                                    true
//                            )
//                    )
//            );
//        }
//
//        if(gp1.ps || gp1.touchpad) {
//            runningActions = new ArrayList<>();
//
//        }


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


        if(gp2.dpad_right()) {
            headingOffset -= 2;
        }
        if(gp2.dpad_left()) {
            headingOffset += 2;
        }

        if(autoDriving) {
            lastHeading = g.getPose().getHeading(); // only set lastHeading once heading lock re-engages
        }
        targetHeading = gamepad1.right_bumper;
        double gpx = goalPose.getX();
        double gpy = goalPose.getY();

        xvelo = insertShiftArray(xvelo, g.follower.getVelocity().getXComponent(), 0);
        yvelo = insertShiftArray(yvelo, g.follower.getVelocity().getYComponent(), 0);

        double accelX = -(xvelo[loopsBack]-xvelo[0])/(loopsBack);
        double accelY = -(yvelo[loopsBack]-yvelo[0])/(loopsBack);

        double xVelPred = xvelo[0]+accelX*loopsPredicted;
        double yVelPred = yvelo[0]+accelY*loopsPredicted;

        gpx = goalPose.getX()-xVelPred*shotTime;
        gpy = goalPose.getY()-yVelPred*shotTime;

        g.currentDistance = g.getPose().distanceFrom(new Pose(goalPose.getX()-xVelPred*shotTime*2, goalPose.getY()-yVelPred*shotTime*2));



        boolean usingApriltag = false;
        AprilTagDetection tag = null;
        if(targetHeading) {
//                if(!g.detections.isEmpty() && Math.abs(g.follower.getVelocity().getXComponent()) < 10 && Math.abs(g.follower.getVelocity().getYComponent()) < 10) {
//                    if (GreenFN.side == RED) {
//                        for (AprilTagDetection detection : g.detections) {
//                            telemetry.addData("detection id: ", detection.id);
//                            if (detection.id == 24) {
//                                usingApriltag = true;
//                                tag = detection;
//                                double distance = g.getPose().distanceFrom(goalPose);
//                                double idealBearing = Math.asin((cameraRedOffset)/(distance-cameraDist));
//                                lastHeading = g.follower.getHeading() - Math.toRadians(detection.ftcPose.bearing)-idealBearing;
////                                hp = 0.5;
////                                headingPIDF.setCoefficients(new PIDFCoefficients(hp, hi, hd, hf));
////                                headingOffset = lastHeading - Math.atan2(gpy - g.getPose().getY(), gpx - g.getPose().getX()) + Math.toRadians(90);
//                            }
//                        }
//                    } else if (GreenFN.side == BLUE) {
//                        for (AprilTagDetection detection : g.detections) {
//                            if (detection.id == 20) {
//                                usingApriltag = true;
//                                tag = detection;
////                                hp = 0.5;
////                                headingPIDF.setCoefficients(new PIDFCoefficients(hp, hi, hd, hf));
//                                double distance = g.getPose().distanceFrom(goalPose);
//                                double idealBearing = Math.asin((cameraBlueOffset)/(distance-cameraDist));
//                                lastHeading = g.follower.getHeading() - Math.toRadians(detection.ftcPose.bearing)-idealBearing;//                                headingOffset = lastHeading - Math.atan2(gpy - g.getPose().getY(), gpx - g.getPose().getX()) + Math.toRadians(90);
//                            }
//                        }
//                    }
//                }
//            else {
//                    hp = 1;
//                    headingPIDF.setCoefficients(new PIDFCoefficients(hp, hi, hd, hf));
//                if(Math.abs(xVelPred)<10&&Math.abs(yVelPred)<10) {
//                    lastHeading = Math.atan2(gpy-g.getPose().getY(), gpx-g.getPose().getX())+Math.toRadians(90)+Math.toRadians(headingOffset);
//
//                }
//                else{
                    lastHeading = Math.atan2(gpy-g.getPose().getY(), gpx-g.getPose().getX())+Math.toRadians(90)+Math.toRadians(headingOffset);
//                }
//            }
        }

        if(gamepad1.dpad_up) {
            double currentHeadingDeg = Math.toDegrees(MathFunctions.normalizeAngle(g.getPose().getHeading()));
            double snappedDeg = Math.round(currentHeadingDeg / 90.0) * 90.0;
            lastHeading = MathFunctions.normalizeAngle(Math.toRadians(snappedDeg));
            targetHeading = true;
        }

        if(gamepad1.left_bumper) {
            lastHeading = Math.atan2(startPose.getY()-g.getPose().getY(), startPose.getX()-g.getPose().getX());
            g.loadStoppers();
//            GreenFN.shooterTarget = GreenFN.shooterCloseRPM;
            targetHeading = true;
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
                headingPower = -gamepad1.right_stick_x * sprint;
//                sprint = 1;
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
                secondaryHeadingPIDF.updateFeedForwardInput(MathFunctions.getTurnDirection(g.getPose().getHeading(), lastHeading));
                secondaryHeadingPIDF.updateError(headingError);
                headingPower = headingPIDF.run();
                if(headingError < Math.PI / 20) {
                    headingPower = secondaryHeadingPIDF.run();
                }
            }

            g.follower.setTeleOpDrive(-gamepad1.left_stick_y  * sprint, -gamepad1.left_stick_x * sprint * 1.27f, headingPower, false);
            g.fifthWheel.setPower(g.follower.getTeleopDriveVector().getYComponent());
        }



        runningActions = newActions;



        telemetry.addData("Current Motif: ", GreenFN.currentMotif);
        telemetry.addData("side: ", GreenFN.side);
        telemetry.addData("intake velocity: ", g.iv);
        telemetry.addData("intake target: ", GreenFN.intakeTarget);
        telemetry.addData("intake power: ", GreenFN.intakePower);
        telemetry.addData("x velo: ", g.follower.getVelocity().getXComponent());
        telemetry.addData("y velo: ", g.follower.getVelocity().getYComponent());
        telemetry.addData("yvelpred: ", yVelPred);
        telemetry.addData("xvelpred: ", xVelPred);
        telemetry.addData("xaccel: ", accelX);
        telemetry.addData("yaccel: ", accelY);
        telemetry.addData("adjusted goal pose x: ", gpx);
        telemetry.addData("adjusted goal pose y: ", gpy);
        telemetry.addData("heading offset: ", headingOffset);
        telemetry.addData("using apriltag: ", usingApriltag);
        telemetry.addData("apriltag bearing: ", usingApriltag?tag.ftcPose.bearing:null);
        telemetry.addData("detections: ", g.detections);



        telemetry.addData("gamepad1 left trigger: ", gamepad1.left_trigger);
        telemetry.addData("gamepad1 left_trigger(): ", gp1.left_trigger());
        telemetry.addData("heading offset: ", headingOffset);

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
        telemetry.addData("side: ", side);
        telemetry.addData("range" , goal==null?null:(goal.ftcPose.range));
        telemetry.addData("pitch: ", goal==null?null:goal.ftcPose.pitch);
        telemetry.addData("pose range: ", g.getPose().distanceFrom(goalPose));
//        telemetry.addData("myNewX: ", myNewX);
//        telemetry.addData("myNewY: ", myNewY);
        telemetry.addData("shotTime: ", shotTime);
        telemetry.addData("rpm reg: ", GreenFN.getRPMreg( g.getPose().distanceFrom(goalPose)));
        telemetry.addData("range adjustment: ", goal==null?null: goal.ftcPose.range*Math.cos(Math.toRadians(goal.ftcPose.pitch)));
        telemetry.addData("adjustedx: ", g.getPoseFromApriltag().getX());
        telemetry.addData("adjustedy: ", g.getPoseFromApriltag().getY());
        lastLoopTime = loopTimer.getElapsedTimeSeconds();
        telemetry.addData("slot1: ", GreenFN.ballColors[0]);
        telemetry.addData("slot2: ", GreenFN.ballColors[1]);
        telemetry.addData("slot3: ", GreenFN.ballColors[2]);

        telemetry.update();
    }
}
