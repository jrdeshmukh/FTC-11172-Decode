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
import com.pedropathing.VectorCalculator;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.GreenFN;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;


@Autonomous()
@Config()
public class TestingGateIntake extends LinearOpMode {

    GreenFN g;

    public class FollowerUpdate implements Action {
        boolean showHeadingVector = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            g.update();
            telemetry.addData("x: ", g.getPose().getX());
            telemetry.addData("y: ", g.getPose().getY());
            telemetry.addData("heading: ", Math.toDegrees(g.getPose().getHeading()));
            ArrayList<Integer> ids = new ArrayList<>();
            for(AprilTagDetection det : g.getApriltagDetections()) {
                ids.add(det.id);
            }
            telemetry.addData("detections: ", ids);
            if(g.follower.isBusy()) {
                showHeadingVector = true;
            }
            if(showHeadingVector) {
                telemetry.addData("heading power: ", g.follower.vectorCalculator.getHeadingVector().getMagnitude());
            }
            telemetry.addData("use secondary: ", VectorCalculator.useSecondaryHeadingPID);
            telemetry.addData("follower isbusy: ", g.follower.isBusy());
            telemetry.addData("t value: ", g.follower.getCurrentTValue());
            telemetry.addData("recentServoShot: ", g.recentServoShot);
            telemetry.addData("shooterPower: ", g.flywheel1Power);
            telemetry.addData("shooterTarget: ", g.shooterTarget);
            telemetry.addData("shooter queue: ", g.shooterQueue.toString());
            telemetry.addData("shooter velo: ", g.s1v);
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
                            g.shoot(3);
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

    class EnableSecondary implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FollowerConstants constants = Constants.followerConstants;
            constants.useSecondaryHeadingPIDF = true;
            g.follower.vectorCalculator.setConstants(constants);
            return false;
        }
    }
    class DisableSecondary implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FollowerConstants constants = Constants.followerConstants;
            constants.useSecondaryHeadingPIDF = false;
            g.follower.vectorCalculator.setConstants(constants);
            return false;
        }
    }
    public Action disableSecondary() {
        return new DisableSecondary();
    }
    public Action enableSecondary() {
        return new EnableSecondary();
    }
    public Action coverDown() {
        return new InstantAction(() -> {
            g.cover1.setPosition(GreenFN.cover1down);
            g.cover2.setPosition(GreenFN.cover2down);
        });
    }
    public Action coverUp() {
        return new InstantAction(() -> {
            g.cover1.setPosition(GreenFN.cover1up);
            g.cover2.setPosition(GreenFN.cover2up);
        });
    }
    public Action sleepTime() {
        return new SleepAction(GreenFN.minTimeBtwnShots*3+0.2);
    }

    public static Pose startPose = new Pose(15, 78.5, Math.toRadians(0));
    public static double intakeHeading = 270;
    public static double intakeTurnEndTime = 0.7;
    public static double gateWait = 2;

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        GreenFN.currentMotif=null;
        GreenFN.closeAuto = true;


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        GreenFN.minTimeBtwnShots = 0.7;
        GreenFN.side = GreenFN.Sides.BLUE;
        g.loadStoppers();
        g.follower.setStartingPose(startPose);
        g.follower.setPose(startPose);
        g.inAuto = true;
//        VectorCalculator.headingPIDFSwitch = 0.262;

        Path gate =
                new Path(
                        new BezierCurve(
                                startPose,
                                new Pose(34.116, 71.759),
                                new Pose(16.885, 63.038)
                        )
                );
        gate.setConstantHeadingInterpolation(Math.toRadians(0));

        Path path2 =
                new Path(
                        new BezierCurve(
                                new Pose(16.885, 63.038),
                                new Pose(17.533, 58.310),
                                new Pose(14.181, 57.436)
                        )
                );
        //path2.setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(270));

        Path path3 =
                new Path(
                        new BezierCurve(
                                new Pose(14.181, 57.436),
                                new Pose(9.013, 57.321)
                        )
                );
        //path3.setConstantHeadingInterpolation(Math.toRadians(270));

        PathChain intake =
                new PathChain(
                        path2,
                        path3
                );
        intake.setHeadingInterpolator(HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(intakeHeading), intakeTurnEndTime));

        GreenFN.detect = true;
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
                                followUntilDone(gate),
                                new SleepAction(gateWait),
                                followUntilDoneChain(intake)
                        )
                )
        );

    }

}