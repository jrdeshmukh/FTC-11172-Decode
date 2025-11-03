package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.util.List;


@Autonomous()
@Config()
@Disabled
public class RedAuto extends LinearOpMode {

    GreenFN g;

    public class FollowerUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            g.update();
            telemetry.addData("x: ", g.getPose().getX());
            telemetry.addData("y: ", g.getPose().getY());
            telemetry.addData("heading: ", Math.toDegrees(g.getPose().getHeading()));
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



    public Action shootInOrder(GreenFN.Motifs motif, int row) {
        switch(motif) {
            case GPP:
                switch(row) {
                    case 0:
                        return new InstantAction(() ->
                        {
                            g.shoot(3);
                            g.shoot(1);
                            g.shoot(2);
                        });
                    case 1:
                        return new InstantAction(() ->
                        {
                            g.shoot(3);
                            g.shoot(2);
                            g.shoot(1);
                        });
                    case 2:
                        return new InstantAction(() ->
                        {
                            g.shoot(3);
                            g.shoot(1);
                            g.shoot(2);
                        });
                    case 3:
                        return new InstantAction(() ->
                        {
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                        });
                }
                break;
            case PGP:
                switch(row) {
                    case 0:
                        return new InstantAction(() -> {
                            g.shoot(3);
                            g.shoot(2);
                            g.shoot(1);
                        });
                    case 1:
                        return new InstantAction(() -> {
                            g.shoot(3);
                            g.shoot(1);
                            g.shoot(2);
                        });
                    case 2:
                        return new InstantAction(() -> {
                            g.shoot(3);
                            g.shoot(2);
                            g.shoot(1);
                        });
                    case 3:
                        return new InstantAction(() -> {
                            g.shoot(1);
                            g.shoot(3);
                            g.shoot(2);
                        });
                }
                break;
            case PPG:
                switch(row) {
                    case 0:
                        return new InstantAction(() -> {
                            g.shoot(2);
                            g.shoot(3);
                            g.shoot(1);
                        });
                    case 1:
                        return new InstantAction(() -> {
                            g.shoot(1);
                            g.shoot(3);
                            g.shoot(2);
                        });
                    case 2:
                        return new InstantAction(() -> {
                            g.shoot(2);
                            g.shoot(3);
                            g.shoot(1);
                        });
                    case 3:
                        return new InstantAction(() -> {
                            g.shoot(3);
                            g.shoot(2);
                            g.shoot(1);
                        });
                }
                break;
        }
        return new InstantAction(()-> g.shoot(1));
    }
    public static Pose startPose = BlueAuto.startPose.mirror();
    public static Pose launchPose = BlueAuto.launchPose.mirror();

    public static Pose aprilTagPose = BlueAuto.aprilTagPose.mirror();

    public static Pose gotoFirstCtrl = BlueAuto.gotoFirstCtrl.mirror();
    public static Pose firstIntakePose = BlueAuto.firstIntakePose.mirror();
    public static Pose firstIntakeEnd = BlueAuto.firstIntakeEnd.mirror();

    public static Pose secondGotoPose = BlueAuto.secondGotoPose.mirror();
    public static Pose secondIntakePose = BlueAuto.secondIntakePose.mirror();

    public static Pose thirdGotoPose = BlueAuto.thirdGotoPose.mirror();
    public static Pose thirdIntakePose = BlueAuto.thirdIntakePose.mirror();

    public static Pose parkPose = BlueAuto.parkPose.mirror();


    @Override
    public void runOpMode() {
        g = new GreenFN(hardwareMap);
        g.follower.setStartingPose(startPose);

        Path aprilTag =
                new Path(
                        new BezierLine(startPose, aprilTagPose)
                );
        aprilTag.setConstantHeadingInterpolation(aprilTagPose.getHeading());

        Path scorePreload =
                new Path(
                        new BezierLine(aprilTagPose, launchPose)
                );
        scorePreload.setLinearHeadingInterpolation(aprilTagPose.getHeading(), launchPose.getHeading());

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

        Path scoreThird =
                new Path(
                        new BezierLine(thirdIntakePose, launchPose)
                );
        scoreThird.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(223));

        Path park =
                new Path(
                        new BezierLine(launchPose, parkPose)
                );
        park.setTangentHeadingInterpolation();




        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
//                                g.setIntakeRPM((int) GreenFN.intakeRPM),
//                                g.setShooterRPM((int) GreenFN.shooterCloseRPM),
                                g.loadAction(),
                                followUntilDone(aprilTag),
                                new SleepAction(0.5),
                                followUntilDone(scorePreload),
                                shootInOrder(GreenFN.currentMotif, 0),
                                new SleepAction(4),
                                g.loadAction(),
                                followUntilDone(gotoFirst),
                                followUntilDone(intakeFirst),
                                new SleepAction(0.5),
                                followUntilDone(scoreFirst),
                                shootInOrder(GreenFN.currentMotif, 1),
                                new SleepAction(4),
                                g.loadAction(),
                                followUntilDone(gotoSecond),
                                followUntilDone(intakeSecond),
                                new SleepAction(0.5),
                                followUntilDone(scoreSecond),
                                shootInOrder(GreenFN.currentMotif, 2),
                                new SleepAction(4),
                                g.loadAction(),
                                followUntilDone(gotoThird),
                                followUntilDone(intakeThird),
                                new SleepAction(0.5),
                                followUntilDone(scoreThird),
                                shootInOrder(GreenFN.currentMotif, 3),
                                followUntilDone(park)
                        )
                )
        );

    }

}
