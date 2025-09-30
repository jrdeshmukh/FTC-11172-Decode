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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GreenFN;


@Autonomous()
@Config()
public class BlueAuto extends LinearOpMode {

    GreenFN g;

    public class FollowerUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            g.update();
            telemetry.addData("x: ", g.follower.getPose().getX());
            telemetry.addData("y: ", g.follower.getPose().getY());
            telemetry.addData("heading: ", Math.toDegrees(g.follower.getPose().getHeading()));
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

    @Override
    public void runOpMode() {
        g = new GreenFN(hardwareMap);
        Pose startPose = new Pose(7.5, 62, Math.toRadians(180));
        g.follower.setStartingPose(startPose);

        PathBuilder builder = new PathBuilder(g.follower);

        PathChain apriltag = builder
                .addPath(
                        new BezierLine(new Pose(24.070, 129.657), new Pose(45.585, 119.277))
                )
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        PathChain scorePreload = builder
                .addPath(
                        new BezierLine(new Pose(45.585, 119.277), new Pose(60.000, 78.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(39))
                .build();

        PathChain gotoFirst = builder
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 78.000),
                                new Pose(41.811, 113.237),
                                new Pose(23.882, 97.384)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(39), Math.toRadians(180))
                .build();

        PathChain intakeFirst = builder
                .addPath(new BezierLine(new Pose(23.882, 97.384), new Pose(24.070, 89.835)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain shootFirst = builder
                .addPath(new BezierLine(new Pose(24.070, 89.835), new Pose(60.000, 78.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(39))
                .build();

        PathChain gotoSecond = builder
                .addPath(new BezierLine(new Pose(60.000, 78.000), new Pose(24.070, 70.773)))
                .setLinearHeadingInterpolation(Math.toRadians(39), Math.toRadians(180))
                .build();

        PathChain intakeSecond = builder
                .addPath(new BezierLine(new Pose(24.070, 70.773), new Pose(24.070, 65.489)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain shootSecond = builder
                .addPath(new BezierLine(new Pose(24.070, 65.489), new Pose(60.000, 78.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(39))
                .build();

        PathChain park = builder
                .addPath(new BezierLine(new Pose(60.000, 78.000), new Pose(54.078, 63.413)))
                .setTangentHeadingInterpolation()
                .build();

        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
                                followUntilDoneChain(apriltag),
                                new SleepAction(0.5),
                                followUntilDoneChain(scorePreload)
                        )
                )
        );

    }

}
