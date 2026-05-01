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
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.util.List;


@Autonomous()
@Config()
@Disabled()

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
                            g.shoot(3);
                            g.shoot(2);
                            g.shoot(1);
                            break;
                        case 2:
                            g.shoot(2);
                            g.shoot(3);
                            g.shoot(1);
                            break;
                        case 3:
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
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
                            g.shoot(3);
                            g.shoot(1);
                            break;
                        case 2:
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                            break;
                        case 3:
                            g.shoot(2);
                            g.shoot(1);
                            g.shoot(3);
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
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                            break;
                        case 2:
                            g.shoot(1);
                            g.shoot(3);
                            g.shoot(2);
                            break;
                        case 3:
                            g.shoot(3);
                            g.shoot(2);
                            g.shoot(1);
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

    // New Pose Definitions based on the new paths

    // START POSE (from Path1 start)
    public static Pose startPose = new Pose(57.098, 9.248, Math.toRadians(180));

    // LAUNCH POSE (from Path1 end, Path4 end, Path7 end, Path8 start)
    public static double launchPoseX = 59.000, launchPoseY = 84.000, launchPoseHeading = Math.toRadians(226);
    public static Pose launchPose = new Pose(launchPoseX, launchPoseY, launchPoseHeading);

    // INTERMEDIATE POSES (from the new path coordinates)

    // Poses for First Intake (from Path2 & Path3)
    public static Pose firstIntakeCtrl = new Pose(56.532, 11.512); // P2 control point
    public static Pose firstIntakeStart = new Pose(24.259, 21.704); // P2 end / P3 start
    public static Pose firstIntakeEnd = new Pose(23.882, 30.952); // P3 end / P4 start
    public static Pose firstScoreCtrl = new Pose(58.608, 49.258); // P4 control point

    // Poses for Second Intake (from Path5 & Path6)
    public static Pose secondIntakeCtrl = new Pose(50.870, 41.520); // P5 control point
    public static Pose secondIntakeStart = new Pose(17.087, 9.436); // P5 end / P6 start
    public static Pose secondIntakeEnd = new Pose(9.349, 9.059); // P6 end / P7 start
    public static Pose secondScoreCtrl = new Pose(56.909, 52.089); // P7 control point

    // Park Pose (from Path8 end)
    public static Pose parkPose = new Pose(59.363, 61.526);

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        g.loadStoppers();
        g.follower.setStartingPose(startPose);

        // Path 1 -> scorePreload
        Path scorePreload =
                new Path(
                        new BezierLine(startPose, launchPose)
                );
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        // Path 2 -> gotoIntake1
        Path gotoIntake1 =
                new Path(
                        new BezierCurve(launchPose, firstIntakeCtrl, firstIntakeStart)
                );
        gotoIntake1.setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(180));

        // Path 3 -> intake1
        Path intake1 =
                new Path(
                        new BezierLine(firstIntakeStart, firstIntakeEnd)
                );
        intake1.setConstantHeadingInterpolation(Math.toRadians(180));

        // Path 4 -> score1
        Path score1 =
                new Path(
                        new BezierCurve(firstIntakeEnd, firstScoreCtrl, launchPose)
                );
        score1.setLinearHeadingInterpolation(Math.toRadians(180), launchPose.getHeading());

        // Path 5 -> gotoIntake2
        Path gotoIntake2 =
                new Path(
                        new BezierCurve(launchPose, secondIntakeCtrl, secondIntakeStart)
                );
        gotoIntake2.setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(-90));

        // Path 6 -> intake2
        Path intake2 =
                new Path(
                        new BezierLine(secondIntakeStart, secondIntakeEnd)
                );
        intake2.setConstantHeadingInterpolation(Math.toRadians(-90));

        // Path 7 -> score2
        Path score2 =
                new Path(
                        new BezierCurve(secondIntakeEnd, secondScoreCtrl, launchPose)
                );
        score2.setLinearHeadingInterpolation(Math.toRadians(-90), launchPose.getHeading());

        // Path 8 -> park
        Path park =
                new Path(
                        new BezierLine(launchPose, parkPose)
                );
        park.setTangentHeadingInterpolation();


        GreenFN.detect = false;
        GreenFN.intakeTarget = 0;
        GreenFN.shooterTarget = 0;
        g.loadStoppers();


        while(!isStarted()) {
            g.update();
        }
        if(isStopRequested()) return;

        // --- Autonomous Sequence using new paths ---
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
                                // Initial setup
                                g.setShooterRPM((int) 680),
                                g.loadAction(),

                                // Score Preload
                                followUntilDone(scorePreload), // Move to launchPose
                                g.loadAction(),
                                new InstantAction(() -> GreenFN.detect = true),
                                shootInOrder(0),
                                new SleepAction(GreenFN.minTimeBtwnShots * 3),
                                g.loadAction(),

                                // First Intake Run and Score
                                followUntilDone(gotoIntake1), // Move towards intake zone
                                new InstantAction(() -> g.setIntakeRPM((int) GreenFN.intakeRPM+250)),
                                followUntilDone(intake1), // Intake path
                                new SleepAction(0.2), // Wait for intake
                                followUntilDone(score1), // Move back to launchPose
                                shootInOrder(1),
                                new SleepAction(GreenFN.minTimeBtwnShots * 3),
                                g.loadAction(),

                                // Second Intake Run and Score
                                followUntilDone(gotoIntake2), // Move towards intake zone
                                followUntilDone(intake2), // Intake path
                                new SleepAction(0.2), // Wait for intake
                                followUntilDone(score2), // Move back to launchPose
                                shootInOrder(2),
                                new SleepAction(GreenFN.minTimeBtwnShots * 3),
                                g.loadAction(),

                                // Park
                                followUntilDone(park)
                        )
                )
        );

    }

}