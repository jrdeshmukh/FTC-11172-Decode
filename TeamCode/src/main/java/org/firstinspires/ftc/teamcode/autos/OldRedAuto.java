package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.OldBlueAuto.launchPoseHeading;
import static org.firstinspires.ftc.teamcode.autos.OldBlueAuto.launchPoseX;
import static org.firstinspires.ftc.teamcode.autos.OldBlueAuto.launchPoseY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.VectorCalculator;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.util.List;
import java.util.Vector;


@Autonomous()
@Config()
public class OldRedAuto extends LinearOpMode {

    GreenFN g;

    public class FollowerUpdate implements Action {
        boolean showHeadingVector = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            g.update();
            telemetry.addData("x: ", g.getPose().getX());
            telemetry.addData("y: ", g.getPose().getY());
            telemetry.addData("heading: ", Math.toDegrees(g.getPose().getHeading()));
            if(g.follower.isBusy()) {
                showHeadingVector = true;
            }
            if(showHeadingVector) {
                telemetry.addData("heading power: ", g.follower.vectorCalculator.getHeadingVector().getMagnitude());
            }
            telemetry.addData("recentServoShot: ", g.recentServoShot);
            telemetry.addData("shooterPower: ", g.flywheel1Power);
            telemetry.addData("shooterTarget: ", g.shooterTarget);
            telemetry.addData("shooter queue: ", g.shooterQueue.toString());
            telemetry.addData("motif: ", GreenFN.currentMotif);

            telemetry.update();
            return true;
        }
    }

    public Action follow(Path a) {
        return new InstantAction(() -> {
            g.follower.breakFollowing();
            g.follower.followPath(a, true);
        }
        );
    }

    public Action follow(PathChain c) {
        return new InstantAction(() -> {
            g.follower.breakFollowing();
            g.follower.followPath(c, true);
        }
        );
    }

    public Action follow(PathChain c, boolean holdEnnd) {
        return new InstantAction(() -> {
            g.follower.breakFollowing();
            g.follower.followPath(c, holdEnnd);
        }
        );
    }

    public Action followUntilDone(Path a) {
        return new FollowUntilDone(a);
    }

    public Action followUntilDone(Path a, boolean holdEnd) {
        return new FollowUntilDone(a, holdEnd);
    }

    public Action followUntilDoneChain(PathChain a) {
        return new FollowUntilDoneChain(a);
    }

    public Action followUntilDoneChain(PathChain a, boolean holdEnd) {
        return new FollowUntilDoneChain(a, false);
    }

    public class FollowUntilDone implements Action {

        Path a;
        boolean called = false;
        boolean holdEnd = true;


        public FollowUntilDone(Path a)  {
            this.a = a;
        }

        public FollowUntilDone(Path a, boolean holdEnd)  {
            this.a = a;
            this.holdEnd = holdEnd;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!called) {
                g.follower.breakFollowing();
                g.follower.followPath(a, holdEnd);
                called = true;
            }
            return g.follower.isBusy();
        }
    }

    public class WaitToDone implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return g.follower.isBusy();
        }
    }

    public class FollowUntilDoneChain implements Action {

        PathChain a;
        boolean called = false;
        boolean holdEnd = true;

        public FollowUntilDoneChain(PathChain a)  {
            this.a = a;
        }
        public FollowUntilDoneChain(PathChain a, boolean holdEnd) {
            this.a = a;
            this.holdEnd = holdEnd;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!called) {
                g.follower.breakFollowing();
                g.follower.followPath(a, holdEnd);
                called = true;
            }
            return g.follower.isBusy();
        }
    }
    public class ShootInOrder implements Action {

        int row;

        public ShootInOrder(int row) {
            this.row = row;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            GreenFN.Motifs motif = g.getCurrentMotif();
            if (motif==null) {
                motif = GreenFN.currentMotif;
            }
            if(motif==null) {
                motif = GreenFN.Motifs.PGP;
            }
            switch(motif) {
                case GPP:
                    switch(row) {
                        case 0:
                            g.shoot(2);
                            g.shoot(3);
                            g.shoot(1);
                            break;
                        case 1:
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                            break;
                        case 2:
                            g.shoot(2);
                            g.shoot(1);
                            g.shoot(3);
                            break;
                        case 3:
                            g.shoot(3);
                            g.shoot(2);
                            g.shoot(1);
                            break;
                    }
                    break;
                case PGP:
                    switch(row) {
                        case 0:
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                            break;
                        case 1:
                            g.shoot(2);
                            g.shoot(1);
                            g.shoot(3);
                            break;
                        case 2:
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                            break;
                        case 3:
                            g.shoot(2);
                            g.shoot(3);
                            g.shoot(1);
                            break;
                    }
                    break;
                case PPG:
                    switch(row) {
                        case 0:
                            g.shoot(1);
                            g.shoot(3);
                            g.shoot(2);
                            break;
                        case 1:
                            g.shoot(2);
                            g.shoot(3);
                            g.shoot(1);
                            break;
                        case 2:
                            g.shoot(1);
                            g.shoot(3);
                            g.shoot(2);
                            break;
                        case 3:
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                            break;
                    }
                    break;
            }
            return false;
        }
    }
    public Action shootInOrder(int row) {
        return new ShootInOrder(row);
    }

    public class ShootStrafe implements Action {

        Pose translatorPose = new Pose(0, 5.70, 0);
        boolean ran = false;

        int shooter;
        public ShootStrafe(int shooter) {
            this.shooter = shooter;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!ran) {
                translatorPose = translatorPose.rotate(follower.getPose().getHeading(), true);
                Pose[] shooterPoses = {new Pose(0,0,0), g.follower.getPose().plus(translatorPose), g.follower.getPose(), g.follower.getPose().minus(translatorPose)};
                Path shooterPath = new Path(
                        new BezierLine(
                                g.follower.getPose(),
                                shooterPoses[shooter]
                        )
                );
                shooterPath.setConstantHeadingInterpolation(g.follower.getPose().getHeading());
                g.follower.followPath(
                        shooterPath
                );
            }
            if(!g.follower.isBusy()) {
                g.shoot(shooter);
            }
            return g.follower.isBusy();
        }
    }
    public Action shootStrafe(int shooter) {
        return new ShootStrafe(shooter);
    }

//    X: : 33.92187403881643
//    Y: 96.10909048966536
//    apriltagheading: 0.0
//    apriltagx: 0.0
//    apriltagy: 0.0
//    heading: : -2.627474546432495

    public static Pose mirrorNoHeading(Pose in) {
        return new Pose(144-in.getX(), in.getY(), in.getHeading());
    }

    public static Pose startPose = new Pose(144-15.0, 118.0, Math.toRadians(218));
    //    public static double launchPoseX = 33.92, launchPoseY = 96.11, launchPoseHeading = Math.toRadians(209.484);
    public static double launchheading = Math.toRadians(126);
    public static Pose launchPose = new Pose(144-launchPoseX, launchPoseY, launchheading);
    public static Pose aprilTagPoseOld = new Pose(46.008, 96.996, Math.toRadians(160));
    public static Pose aprilTagPose = aprilTagPoseOld.mirror().setHeading(Math.toRadians(200));

    public static Pose gotoFirstCtrl = mirrorNoHeading(OldBlueAuto.gotoFirstCtrl);
    public static Pose firstIntakePose = mirrorNoHeading(OldBlueAuto.firstIntakePose);
    public static Pose firstIntakeEnd = mirrorNoHeading(OldBlueAuto.firstIntakeEnd);

    public static Pose secondGotoPose = mirrorNoHeading(OldBlueAuto.secondGotoPose);
    public static Pose secondIntakePose = mirrorNoHeading(OldBlueAuto.secondIntakePose);

    public static Pose thirdGotoPose = mirrorNoHeading(OldBlueAuto.thirdGotoPose);
    public static Pose thirdIntakePose = mirrorNoHeading(OldBlueAuto.thirdIntakePose);

    public static Pose parkPose = mirrorNoHeading(OldBlueAuto.parkPose).setHeading(launchPose.getHeading());

    public Action disableSecondary() {
        return new InstantAction(() -> VectorCalculator.useSecondaryHeadingPID = false);
    }
    public Action enableSecondary() {
        return new InstantAction(() -> VectorCalculator.useSecondaryHeadingPID = true);
    }

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        GreenFN.currentMotif=null;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        GreenFN.minTimeBtwnShots = 0.7;
        GreenFN.side = GreenFN.Sides.RED;
        g.loadStoppers();
        g.follower.setStartingPose(startPose);
        g.follower.setPose(startPose);
        g.inAuto = true;
//        VectorCalculator.headingPIDFSwitch = 0.262;


        Path aprilTag =
                new Path(
                        new BezierLine(startPose, aprilTagPose)
                );
        aprilTag.setLinearHeadingInterpolation(startPose.getHeading(), aprilTagPose.getHeading());

        Path scorePreload =
                new Path(
                        new BezierLine(aprilTagPose, launchPose)
                );
        scorePreload.setConstantHeadingInterpolation(launchPose.getHeading());

        PathChain aprilTagPreload = new PathChain(
                aprilTag,
                scorePreload
        );

        Path gotoFirst =
                new Path(
                        new BezierCurve(launchPose, gotoFirstCtrl, firstIntakePose)
                );
        gotoFirst.setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(0));

        Path intakeFirst =
                new Path(
                        new BezierLine(firstIntakePose, firstIntakeEnd)
                );
        intakeFirst.setConstantHeadingInterpolation(Math.toRadians(0));

        PathChain first = new PathChain(
                gotoFirst,
                intakeFirst
        );

        Path scoreFirst =
                new Path(
                        new BezierLine(firstIntakeEnd, launchPose)
                );
        scoreFirst.setLinearHeadingInterpolation(Math.toRadians(0), launchPose.getHeading());

        Path gotoSecond =
                new Path(
                        new BezierLine(launchPose, secondGotoPose)
                );
        gotoSecond.setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(0));

        Path intakeSecond =
                new Path(
                        new BezierLine(secondGotoPose, secondIntakePose)
                );
        intakeSecond.setConstantHeadingInterpolation(Math.toRadians(0));

        PathChain second = new PathChain(
                gotoSecond,
                intakeSecond
        );

        Path scoreSecond =
                new Path(
                        new BezierLine(secondIntakePose, launchPose)
                );
        scoreSecond.setLinearHeadingInterpolation(Math.toRadians(0), launchPose.getHeading());

        Path gotoThird =
                new Path(
                        new BezierLine(launchPose, thirdGotoPose)
                );
        gotoThird.setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(0));

        Path intakeThird =
                new Path(
                        new BezierLine(thirdGotoPose, thirdIntakePose)
                );
        intakeThird.setConstantHeadingInterpolation(Math.toRadians(0));

        PathChain third = new PathChain(
                gotoThird,
                intakeThird
        );

        Path scoreThird =
                new Path(
                        new BezierLine(thirdIntakePose, launchPose)
                );
        scoreThird.setLinearHeadingInterpolation(Math.toRadians(0), launchPose.getHeading());

        Path park =
                new Path(
                        new BezierLine(launchPose, parkPose)
                );
        park.setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(0));


        GreenFN.detect = false;
        GreenFN.intakeTarget = 0;
        GreenFN.shooterTarget = 0;


        g.loadStoppers();


        while(!isStarted()) {
            g.update();
        }

        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-15.6),
                                enableSecondary(),
                                g.loadAction(),
                                follow(aprilTag),
                                new ParallelAction(
                                        new SleepAction(0.5),
                                        g.loadAction(),
                                        new SequentialAction(
                                                new SleepAction(0.2)
//                                                g.setIntakeRPM((int) GreenFN.intakeRPM+250)
                                        )
                                ),
                                g.loadAction(),
                                enableSecondary(),
                                new WaitToDone(),
                                new InstantAction(() -> GreenFN.detect=true),
                                enableSecondary(),
                                followUntilDone(scorePreload),
                                g.loadAction(),
//                                new SleepAction(1),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                new SleepAction(0.2),
                                shootInOrder( 0),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.1),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
//                                g.setIntakeRPM(0),
                                disableSecondary(),
                                g.loadAction(),

                                new ParallelAction(
                                        followUntilDone(gotoFirst),
                                        new SequentialAction(
                                                new SleepAction(0.2),
                                                g.setIntakeRPM((int) GreenFN.intakeRPM-100)
                                        )
                                ),                                g.setIntakeRPM((int) GreenFN.intakeRPM+250),
                                new InstantAction(() -> g.follower.setMaxPower(0.7)),
                                followUntilDone(intakeFirst),
                                new InstantAction(() -> g.follower.setMaxPower(1)),
                                new SleepAction(0.1),
                                enableSecondary(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                follow(scoreFirst),
                                new SleepAction(0.7),
                                g.setIntakeRPM(0),
                                new WaitToDone(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                new SleepAction(0.2),
                                shootInOrder(1),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.1),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
//                                g.setIntakeRPM(0),
                                disableSecondary(),
                                g.loadAction(),
                                new ParallelAction(
                                        followUntilDone(gotoSecond),
                                        new SequentialAction(
                                                new SleepAction(0.2),
                                                g.setIntakeRPM((int) GreenFN.intakeRPM-100)
                                                )
                                        ),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+250),
                                new InstantAction(() -> g.follower.setMaxPower(0.7)),
                                followUntilDone(intakeSecond),
                                new InstantAction(() -> g.follower.setMaxPower(1)),
                                new SleepAction(0.2),
                                enableSecondary(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                follow(scoreSecond),
                                new SleepAction(0.7),
                                g.setIntakeRPM(0),
                                new WaitToDone(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                new SleepAction(0.2),
                                shootInOrder(2),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.1),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
                                g.setIntakeRPM(0),
                                disableSecondary(),
                                g.loadAction(),
                                followUntilDone(gotoThird),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+250),
                                new InstantAction(() -> g.follower.setMaxPower(0.7)),
                                followUntilDone(intakeThird),
                                new InstantAction(() -> g.follower.setMaxPower(1)),
                                new SleepAction(0.2),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                enableSecondary(),
                                follow(scoreThird),
                                new WaitToDone(),
                                new InstantAction(() -> GreenFN.minTimeBtwnShots = 0.2),
                                new SleepAction(0.5),
                                shootInOrder(3),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.1),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
                                g.setIntakeRPM(0),
                                g.setShooterRPM(0),
                                disableSecondary(),
                                followUntilDone(park)
                        )
                )
        );

    }

}