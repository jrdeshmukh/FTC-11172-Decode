package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Color;

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
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GreenFN;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;


@Autonomous()
@Config()
public class BlueAuto2 extends LinearOpMode {

    GreenFN g;
    public class FollowerUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            g.update();
            telemetry.addData("x: ", g.getPose().getX());
            telemetry.addData("y: ", g.getPose().getY());
            telemetry.addData("heading: ", Math.toDegrees(g.getPose().getHeading()));
            telemetry.addData("heading power: ", g.follower.getHeadingVector());
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
                        case 4:
                            g.shoot(2);
                            g.shoot(3);
                            g.shoot(1);
                            break;
                        case 5:
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
                        case 4:
                            g.shoot(1);
                            g.shoot(2);
                            g.shoot(3);
                            break;
                        case 5:
                            g.shoot(1);
                            g.shoot(2);
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
                        case 4:
                            g.shoot(1);
                            g.shoot(3);
                            g.shoot(2);
                            break;
                        case 5:
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
    public static Pose startPose = new Pose(15.0, 118.0, Math.toRadians(141));

//    X: : 33.92187403881643
//    Y: 96.10909048966536
//    apriltagheading: 0.0
//    apriltagx: 0.0
//    apriltagy: 0.0
//    heading: : -2.627474546432495

    //Launch Variables
    public static double shootPreX = 27.284, shootPreY = 106.535, shootPreHeading = Math.toRadians(225);
    public static double shoot1X = 34.627, shoot1Y = 98.643, shoot1Heading = Math.toRadians(225);
    public static double shoot2X = 48.899, shoot2Y = 88.083, shoot2Heading = Math.toRadians(230);
    public static double shoot3X = 53.762, shoot3Y = 12.055, shoot3Heading = Math.toRadians(210);
    public static double shootHumanX = shoot3X, shootHumanY = shoot3Y, shootHumanHeading = shoot3Heading;
    public static double shoot5X = shootHumanX, shoot5Y = shootHumanY, shoot5Heading = shootHumanHeading;

    //Poses

    //Initial part
    public static Pose aprilTagPose = new Pose(shootPreX, shootPreY);
    public static Pose shootPreloadPose = new Pose(shootPreX, shootPreY, shootPreHeading);

    //First part
    public static Pose pickup1Control1 = new Pose(16.887,109.993);
    public static Pose pickup1Control2 = new Pose(27.379, 71.104);
    public static Pose firstRow = new Pose(25.21970705725699,88.29693741677765, Math.toRadians(360));
    public static Pose shoot1Pose = new Pose(shoot1X,shoot1Y, shoot1Heading);
    public static Pose shoot1Control = new Pose(12.671,81.750);

    //Second part
    public static Pose secondRow = new Pose(21.460,62.487, Math.toRadians(360));
    public static Pose shoot2Pose = new Pose(shoot2X,shoot2Y, shoot2Heading);
    public static Pose pickup2Control = new Pose(19.499,91.648);
    //public static Pose shoot2Control = new Pose(23.972, 50.585);
    public static Pose row2Control1 = new Pose(9.400, 98.800);
    public static Pose row2Control2 = new Pose(28.900, 30.300);
    public static Pose row2Control3 = new Pose(13.900, 24.600);
    public static Pose row2Control4 = new Pose(57.000, 87.800);

    //Third part
    public static Pose thirdRow = new Pose(21.807, 38.534, Math.toRadians(360));
    public static Pose shoot3Pose = new Pose(shoot3X, shoot3Y, shoot3Heading);
    public static Pose pickup3Control = new Pose(17.617, 70.231);
    //public static Pose shoot3Control = new Pose(21.991, 30.247);
    public static Pose row3Control1 = new Pose(7.400, 56.900);
    public static Pose row3Control2 = new Pose(31.900, 85.000);
    public static Pose row3Control3 = new Pose(12.100, 6.900);
    public static Pose row3Control4 = new Pose(21.100, 28.100);
    public static Pose row3Control5 = new Pose(27.800, 32.200);

    //Human part
    public static Pose humanRow = new Pose(7.549, 8.140, Math.toRadians(270));
    public static Pose shootHumanPose = new Pose(shootHumanX,shootHumanY, shootHumanHeading);
    public static Pose pickupHumanControl = new Pose(49.568, 7.896);
    public static Pose human1 = new Pose(3.157, 1.109);
    public static Pose human2 = new Pose(2.665, 5.569);
    public static Pose human3 = new Pose(3.801, 14.421);
    public static Pose human4 = new Pose(0.233, 14.155);

    //Fifth part
    public static Pose fifthRow = new Pose(7.3541944074567125, 21.52197070572569, Math.toRadians(270));
    public static Pose shoot5Pose = new Pose(shoot5X, shoot5Y, shoot5Heading);
    public static Pose fifth1 = new Pose(0.481, 29.416);
    public static Pose fifth2 = new Pose(1.262, 23.813);
    public static Pose fifth3 = new Pose(1.032, 10.605);
    public static Pose fifth4 = new Pose(4.718, 15.741);

    //Park part
    public static Pose parkPose = new Pose(27.07723035952064, 10.760319573901466, Math.toRadians(270));

    public Action disableSecondary() {
        return new InstantAction(() -> VectorCalculator.useSecondaryHeadingPID = false);
    }
    public Action enableSecondary() {
        return new InstantAction(() -> VectorCalculator.useSecondaryHeadingPID = true);
    }

//    public static double launchPoseX = 57, launchPoseY = 105, launchPoseHeading = Math.toRadians(246);
//    //    public static double launchPoseX = 33.92, launchPoseY = 96.11, launchPoseHeading = Math.toRadians(209.484);
//    public static Pose launchPose = new Pose(launchPoseX, launchPoseY, launchPoseHeading);
//
//    // Define all poses and headings as variables at the top
//    public static Pose aprilTagPose = new Pose(46.008, 96.996, Math.toRadians(160));
//
//    // Intermediate poses
//    public static Pose gotoFirstCtrl = new Pose(49.360, 98.516);
//    public static Pose firstIntakePose = new Pose(22.500, 100.0);
//    public static Pose firstIntakeEnd = new Pose(21.50, 89.835);
//
//    public static Pose secondGotoPose = new Pose(23.0, 75.773);
//    public static Pose secondIntakePose = new Pose(21.50, 65.489);
//
//    public static Pose thirdGotoPose = new Pose(22, 48.937);
//    public static Pose thirdIntakePose = new Pose(21.800, 41.332);
//
//    public static Pose parkPose = new Pose(launchPoseX-8, 115);

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        g.loadStoppers();
        g.follower.setStartingPose(startPose);
        GreenFN.minTimeBtwnShots = 0.7;

        Path aprilTag =
                new Path(
                        new BezierLine(startPose, aprilTagPose)
                );
        aprilTag.setConstantHeadingInterpolation(Math.toRadians(141));

        Path shootPreload =
                new Path(
                        new BezierLine(aprilTagPose, shootPreloadPose)
                );
        shootPreload.setLinearHeadingInterpolation(aprilTagPose.getHeading(), shootPreloadPose.getHeading(), 0.2);

        Path pickup1 =
                new Path(
                        new BezierCurve(shootPreloadPose, pickup1Control1, pickup1Control2, firstRow)
                );
        pickup1.setLinearHeadingInterpolation(shootPreloadPose.getHeading(), firstRow.getHeading(), 0.3);

        Path shoot1 =
                new Path(
                        new BezierCurve(firstRow, shoot1Pose)
                );
        shoot1.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .2,
                                HeadingInterpolator.constant(Math.toRadians(360))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .2,
                                1,
                                HeadingInterpolator.constant(Math.toRadians(225))
                        )
                )
        );

        Path row2 =
                new Path(
                        new BezierCurve(shoot1Pose,
                                row2Control1, row2Control2, row2Control3, row2Control4,
                                shoot2Pose)
                );
        row2.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .5,
                                HeadingInterpolator.constant(Math.toRadians(360))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .5,
                                1,
                                HeadingInterpolator.constant(Math.toRadians(230))
                        )
                )
        );

        Path row3 =
                new Path(
                        new BezierCurve(shoot2Pose,
                                row3Control1, row3Control2, row3Control3, row3Control4, row3Control5,
                                shoot3Pose)
                );
        row3.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .5,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .5,
                                .65,
                                HeadingInterpolator.constant(Math.toRadians(360))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .65,
                                .85,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .85,
                                1,
                                HeadingInterpolator.constant(Math.toRadians(shoot3Heading))
                        )
                )
        );

        Path rowHuman =
                new Path(
                        new BezierCurve(shoot3Pose,
                                human1, human2, human3, human4,
                                shootHumanPose)
                );
        rowHuman.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .25,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .25,
                                .6,
                                HeadingInterpolator.constant(Math.toRadians(humanRow.getHeading()))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .6,
                                .9,
                                HeadingInterpolator.tangent.reverse()
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .9,
                                1,
                                HeadingInterpolator.constant(Math.toRadians(shootHumanHeading))
                        )
                )
        );

        Path row5 =
                new Path(
                        new BezierCurve(shootHumanPose,
                                fifth1, fifth2, fifth3, fifth4,
                                shoot5Pose)
                );
        row5.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .35,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .35,
                                .55,
                                HeadingInterpolator.constant(Math.toRadians(fifthRow.getHeading()))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .55,
                                .9,
                                HeadingInterpolator.tangent.reverse()
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .9,
                                1,
                                HeadingInterpolator.constant(Math.toRadians(shoot5Heading))
                        )
                )
        );

        Path park =
                new Path(
                        new BezierLine(shoot5Pose, parkPose)
                );
        park.setLinearHeadingInterpolation(shoot5Pose.getHeading(), 0,0.7);

//        PathChain aprilTagPreload = new PathChain(
//                aprilTag,
//                scorePreload
//        );




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
                                new InstantAction(() -> GreenFN.detect=true),
                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-15.6),
                                g.loadAction(),
                                follow(aprilTag),
                                g.loadAction(),
                                new WaitToDone(),
                                follow(shootPreload),
                                new SleepAction(0.3),
                                enableSecondary(),
                                g.loadAction(),
//                                new SleepAction(1),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                new WaitToDone(),
                                shootInOrder( 0),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.35),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
//                                g.setIntakeRPM(0),
                                disableSecondary(),
                                g.loadAction(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+550),
                                follow(pickup1),
                                new SleepAction(0.3),
                                enableSecondary(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                new WaitToDone(),
                                disableSecondary(),
                                follow(shoot1),
                                new SleepAction(0.4),
                                enableSecondary(),
                                new WaitToDone(),
                                shootInOrder(1),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.35),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
//                                g.setIntakeRPM(0),
                                disableSecondary(),
                                g.loadAction(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+550),
                                follow(row2),
//                                new SleepAction(0.1),
                                new SleepAction(0.3),
                                enableSecondary(),
                                new SleepAction(0.3),
                                disableSecondary(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-1000),
                                new WaitToDone(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                shootInOrder(2),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.35),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
                                disableSecondary(),
                                g.loadAction(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+550),
                                follow(row3),
                                new SleepAction(0.6),
//                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                enableSecondary(),
//                                new InstantAction(() -> GreenFN.minTimeBtwnShots = 0.2),
                                new SleepAction(0.2),
                                disableSecondary(),
                                new SleepAction(0.8),
                                enableSecondary(),
                                new WaitToDone(),
                                shootInOrder(3),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.35),
//                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-200),
                                disableSecondary(),
                                g.loadAction(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+550),
                                follow(rowHuman),
                                new SleepAction(0.5),
                                enableSecondary(),
                                new SleepAction(0.2),
                                disableSecondary(),
                                new SleepAction(0.5),
                                enableSecondary(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                new WaitToDone(),
                                shootInOrder(4),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.35),
                                disableSecondary(),
                                g.loadAction(),
                                follow(row5),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+550),
                                new SleepAction(0.4),
                                enableSecondary(),
                                new SleepAction(0.2),
                                disableSecondary(),
                                g.setIntakeRPM((int) GreenFN.intakeRPM-100),
                                new SleepAction(0.5),
                                enableSecondary(),
                                new WaitToDone(),
                                shootInOrder(5),
                                new SleepAction(GreenFN.minTimeBtwnShots *3-0.35),
                                disableSecondary(),
                                followUntilDone(park),
                                g.setIntakeRPM(0),
                                g.setShooterRPM(0)
                        )
                )
        );

    }

}
