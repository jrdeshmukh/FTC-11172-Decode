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
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.util.List;


@Autonomous()
@Config()
public class FarBlueAuto extends LinearOpMode {

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
                                g.shoot(1);
                                g.shoot(3);
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
    // Shared scoring end pose
    public static Pose launchPose = new Pose(64.332, 12.747, Math.toRadians(200));

    // Path 1
    public static Pose startPose = new Pose(57.162, 9.162, Math.toRadians(180));

    // Path 2
    public static Pose gotoSideMid1 = new Pose(49.195, 16.531);
    public static Pose gotoSideEnd = new Pose(15.137, 9.500, Math.toRadians(-90));

    // Path 3
    public static Pose intakeSideEnd = new Pose(10.556, 10, Math.toRadians(-90));

    // Path 4
    public static Pose scoreSideMid = new Pose(38.440, 21.710);

    // Path 5
    public static Pose gotoFirstEnd = new Pose(24.299, 23.701, Math.toRadians(180));

    // Path 6
    public static Pose intakeFirstEnd = new Pose(24.299, 29.079, Math.toRadians(180));

// Path 7
// goes back to launchPose

    // Path 8
    public static Pose gotoSecondEnd = new Pose(24.299, 47.801, Math.toRadians(180));

    // Path 9
    public static Pose intakeSecondEnd = new Pose(24.299, 53.975, Math.toRadians(180));

// Path 10
// goes back to launchPose

    // Path 11
    public static Pose parkEnd = new Pose(39.436, 14.739, Math.toRadians(200));


    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        g.loadStoppers();
        g.follower.setStartingPose(startPose);

        Path scorePreload =
                new Path(
                        new BezierLine(startPose, launchPose)
                );
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

         Path gotoSide =
                new Path(
                        new BezierCurve(launchPose, gotoSideMid1, gotoSideEnd)
                );
        gotoSide.setLinearHeadingInterpolation(launchPose.getHeading(), gotoSideEnd.getHeading());

        Path intakeSide =
                new Path(
                        new BezierLine(gotoSideEnd, intakeSideEnd)
                );
        intakeSide.setConstantHeadingInterpolation(intakeSideEnd.getHeading());

        Path scoreSide =
                new Path(
                        new BezierCurve(intakeSideEnd, scoreSideMid, launchPose)
                );
        scoreSide.setLinearHeadingInterpolation(intakeSideEnd.getHeading(), launchPose.getHeading());

        Path gotoFirst =
                new Path(
                        new BezierLine(launchPose, gotoFirstEnd)
                );
        gotoFirst.setLinearHeadingInterpolation(launchPose.getHeading(), gotoFirstEnd.getHeading());

        Path intakeFirst =
                new Path(
                        new BezierLine(gotoFirstEnd, intakeFirstEnd)
                );
        intakeFirst.setConstantHeadingInterpolation(intakeFirstEnd.getHeading());

        Path scoreFirst =
                new Path(
                        new BezierLine(intakeFirstEnd, launchPose)
                );
        scoreFirst.setLinearHeadingInterpolation(intakeFirstEnd.getHeading(), launchPose.getHeading());

        Path gotoSecond =
                new Path(
                        new BezierLine(launchPose, gotoSecondEnd)
                );
        gotoSecond.setLinearHeadingInterpolation(launchPose.getHeading(), gotoSecondEnd.getHeading());

        Path intakeSecond =
                new Path(
                        new BezierLine(gotoSecondEnd, intakeSecondEnd)
                );
        intakeSecond.setConstantHeadingInterpolation(intakeSecondEnd.getHeading());

        Path scoreSecond =
                new Path(
                        new BezierLine(intakeSecondEnd, launchPose)
                );
        scoreSecond.setLinearHeadingInterpolation(intakeSecondEnd.getHeading(), launchPose.getHeading());

        Path park =
                new Path(
                        new BezierLine(launchPose, parkEnd)
                );
        park.setConstantHeadingInterpolation(parkEnd.getHeading());







        g.loadStoppers();


        while(!isStarted()) {
            g.update();
        }
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
                                g.setShooterRPM((int) GreenFN.shooterFarRPM),
                                g.loadAction(),
                                follow(scorePreload),
                                g.setIntakeRPM((int) GreenFN.intakeRPM),
                                g.loadAction(),
                                new WaitToDone(),
                                g.loadAction(),
//                                new SleepAction(0.5),
                                shootInOrder( 0),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                g.loadAction(),
                                followUntilDone(gotoSide),
                                followUntilDone(intakeSide),
                                new SleepAction(0.2),
                                followUntilDone(scoreSide),
//                                new SleepAction(0.5),
                                shootInOrder(1),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                g.loadAction(),
                                followUntilDone(gotoFirst),
                                followUntilDone(intakeFirst),
                                new SleepAction(0.2),
                                followUntilDone(scoreFirst),
//                                new SleepAction(0.5),
                                shootInOrder(2),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                g.loadAction(),
                                followUntilDone(gotoSecond),
                                followUntilDone(intakeSecond),
                                new SleepAction(0.2),
                                followUntilDone(scoreSecond),
//                                new SleepAction(0.5),
                                shootInOrder(3),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                followUntilDone(park)
                        )
                )
        );

    }

}
