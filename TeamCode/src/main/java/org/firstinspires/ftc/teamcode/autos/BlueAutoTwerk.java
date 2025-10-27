package org.firstinspires.ftc.teamcode.autos;

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
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.util.ArrayList;
import java.util.List;


@Autonomous()
@Config()
public class BlueAutoTwerk extends LinearOpMode {

    GreenFN g;

    public class FollowerUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            g.update();
            telemetry.addData("x: ", g.getPose().getX());
            telemetry.addData("y: ", g.getPose().getY());
            telemetry.addData("heading: ", Math.toDegrees(g.getPose().getHeading()));
            telemetry.addData("recentServoShot: ", g.recentServoShot);
            telemetry.addData("shooterPower: ", g.flywheel1Power);
            telemetry.addData("shooterTarget: ", g.shooterTarget);

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
        ArrayList<Action> initialActions = new ArrayList<>();

        public ShootInOrder(int row) {
            this.row = row;
            GreenFN.Motifs motif = g.getCurrentMotif();
            if (motif == null) {
                motif = GreenFN.currentMotif;
            }
            if (motif == null) {
                motif = GreenFN.Motifs.PGP;
            }
            switch (motif) {
                case GPP:
                    switch (row) {
                        case 0:
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(3));
                            break;
                        case 1:
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(3));
                            break;
                        case 2:
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(3));
                            break;
                        case 3:
                            initialActions.add(shootStrafe(3));
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(1));
                            break;
                    }
                    break;
                case PGP:
                    switch (row) {
                        case 0:
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(3));
                            break;
                        case 1:
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(3));
                            break;
                        case 2:
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(3));
                            break;
                        case 3:
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(3));
                            initialActions.add(shootStrafe(1));
                            break;
                    }
                    break;
                case PPG:
                    switch (row) {
                        case 0:
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(3));
                            initialActions.add(shootStrafe(2));
                            break;
                        case 1:
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(3));
                            initialActions.add(shootStrafe(1));
                            break;
                        case 2:
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(3));
                            initialActions.add(shootStrafe(2));
                            break;
                        case 3:
                            initialActions.add(shootStrafe(1));
                            initialActions.add(shootStrafe(2));
                            initialActions.add(shootStrafe(3));
                            break;
                    }
                    break;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (initialActions.isEmpty()) {
                return false;
            }

            if (initialActions.get(0).run(telemetryPacket)) {
                return true;
            } else {
                initialActions.remove(0);
            }
            return true;
        }
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
                Pose[] shooterPoses = {new Pose(0,0,0), launchPose.plus(translatorPose), launchPose, launchPose.minus(translatorPose)};
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

    public Action shootInOrder(int shooter) {
        return new ShootInOrder(shooter);
    }

    public static Pose middleShootPose = new Pose(59,84, Math.toRadians(230));
    public static Pose rightShootPose = new Pose(56,84, Math.toRadians(227));
    public static Pose leftShootPose = new Pose(59,89, Math.toRadians(220));
    public static Pose startPose = new Pose(15, 118, Math.toRadians(138));
    public static double launchPoseX = 59, launchPoseY = 84, launchPoseHeading = Math.toRadians(230);
    public static Pose launchPose = new Pose(launchPoseX, launchPoseY, launchPoseHeading);

    // Define all poses and headings as variables at the top
    public static Pose aprilTagPose = new Pose(45.585, 119.277, Math.toRadians(-36.5));

    // Intermediate poses
    public static Pose gotoFirstCtrl = new Pose(49.360, 98.516);
    public static Pose firstIntakePose = new Pose(25.00, 100.0);
    public static Pose firstIntakeEnd = new Pose(22.50, 89.835);

    public static Pose secondGotoPose = new Pose(25.00, 75.773);
    public static Pose secondIntakePose = new Pose(25.00, 65.489);

    public static Pose thirdGotoPose = new Pose(24.070, 47.937);
    public static Pose thirdIntakePose = new Pose(24.070, 41.332);

    public static Pose parkPose = new Pose(51.058, 76.058);

    @Override
    public void runOpMode() {
        g = new GreenFN(hardwareMap);
        g.loadStoppers();
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
        g.loadStoppers();

        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
                                g.setShooterRPM((int) GreenFN.shooterCloseRPM),
                                g.loadAction(),
                                followUntilDone(aprilTag),
                                new ParallelAction(
                                        new SleepAction(0.5),
                                        g.loadAction(),
                                        new SequentialAction(
                                                new SleepAction(0.2),
                                                g.setIntakeRPM((int) GreenFN.intakeRPM)
                                        )
                                ),
                                g.loadAction(),
                                followUntilDone(scorePreload),
                                g.loadAction(),
                                new SleepAction(0.5),
                                shootInOrder(0),
                                new SleepAction(g.minTimeBtwnShots*3 + 0.4),
                                g.loadAction(),
                                followUntilDone(gotoFirst),
                                followUntilDone(intakeFirst),
                                new SleepAction(0.5),
                                followUntilDone(scoreFirst),
                                new SleepAction(0.5),
                                shootInOrder(1),
                                new SleepAction(g.minTimeBtwnShots*3 + 0.4),
                                g.loadAction(),
                                followUntilDone(gotoSecond),
                                followUntilDone(intakeSecond),
                                new SleepAction(0.5),
                                followUntilDone(scoreSecond),
                                new SleepAction(0.5),
                                shootInOrder(2),
                                new SleepAction(g.minTimeBtwnShots*3 + 0.4),
                                g.loadAction(),
                                followUntilDone(gotoThird),
                                followUntilDone(intakeThird),
                                new SleepAction(0.5),
                                followUntilDone(scoreThird),
                                new SleepAction(0.5),
                                shootInOrder(3),
                                followUntilDone(park)
                        )
                )
        );

    }

}
