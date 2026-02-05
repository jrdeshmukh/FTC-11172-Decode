package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.BlueAuto.launchPoseX;
import static org.firstinspires.ftc.teamcode.autos.BlueAuto.launchPoseY;
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
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

import java.util.List;


@Autonomous()
@Config()
public class RedAutoTest extends LinearOpMode {

    GreenFN g;

    public class FollowerUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            g.auto = true;
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


            Size myBoxFitSize;
            for(ColorBlobLocatorProcessor.Blob b : g.rightPurple)
            {
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize = boxFit.size;
                telemetry.addData("new purple width", myBoxFitSize.width);
                telemetry.addData("new purple height", myBoxFitSize.height);
                telemetry.addData("new purple angle", boxFit.angle);
                if(myBoxFitSize.width/ myBoxFitSize.height > 3.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.35 ){
                    g.rightBlobs = g.rightBlobs + 1;
                }
                else if (myBoxFitSize.width/ myBoxFitSize.height > 5.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.18) {
                    g.rightBlobs = g.rightBlobs + 2;
                }
                else if (myBoxFitSize.width/ myBoxFitSize.height > 7.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.135) {
                    g.rightBlobs = g.rightBlobs + 3;
                }
                else if (myBoxFitSize.width/ myBoxFitSize.height > 9.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.11) {
                    g.rightBlobs = g.rightBlobs + 4;
                }
                else if (myBoxFitSize.width/ myBoxFitSize.height > 11.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.09) {
                    g.rightBlobs = g.rightBlobs + 5;
                }
                else if (myBoxFitSize.width/ myBoxFitSize.height > 13.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.078) {
                    g.rightBlobs = g.rightBlobs + 6;
                }
            }
            Size myBoxFitSize2;
            for(ColorBlobLocatorProcessor.Blob b : g.rightGreen)
            {
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize2 = boxFit.size;
                telemetry.addData("new green width", myBoxFitSize2.width);
                telemetry.addData("new green height", myBoxFitSize2.height);
                telemetry.addData("new green angle", boxFit.angle);
                if(myBoxFitSize2.width/ myBoxFitSize2.height > 3.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.35 ){
                    g.rightBlobs = g.rightBlobs + 1;
                }
                else if (myBoxFitSize2.width/ myBoxFitSize2.height > 5.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.18) {
                    g.rightBlobs = g.rightBlobs + 2;
                }
                else if (myBoxFitSize2.width/ myBoxFitSize2.height > 7.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.135) {
                    g.rightBlobs = g.rightBlobs + 3;
                }
                else if (myBoxFitSize2.width/ myBoxFitSize2.height > 9.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.11) {
                    g.rightBlobs = g.rightBlobs + 4;
                }
                else if (myBoxFitSize2.width/ myBoxFitSize2.height > 11.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.09) {
                    g.rightBlobs = g.rightBlobs + 5;
                }
                else if (myBoxFitSize2.width/ myBoxFitSize2.height > 13.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.078) {
                    g.rightBlobs = g.rightBlobs + 6;
                }
            }
            Size myBoxFitSize3;
            for(ColorBlobLocatorProcessor.Blob b : g.leftPurple)
            {
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize3 = boxFit.size;
                telemetry.addData("new purple width", myBoxFitSize3.width);
                telemetry.addData("new purple height", myBoxFitSize3.height);
                telemetry.addData("new purple angle", boxFit.angle);
                if(myBoxFitSize3.width/ myBoxFitSize3.height > 3.8 || myBoxFitSize3.width/ myBoxFitSize3.height < 0.35 ){
                    g.leftBlobs = g.leftBlobs + 1;
                }
                else if (myBoxFitSize3.width/ myBoxFitSize3.height > 5.8 || myBoxFitSize3.width/ myBoxFitSize3.height < 0.18) {
                    g.leftBlobs = g.leftBlobs + 2;
                }
                else if (myBoxFitSize3.width/ myBoxFitSize3.height > 7.8 || myBoxFitSize3.width/ myBoxFitSize3.height < 0.135) {
                    g.leftBlobs = g.leftBlobs + 3;
                }
                else if (myBoxFitSize3.width/ myBoxFitSize3.height > 9.8 || myBoxFitSize3.width/ myBoxFitSize3.height < 0.11) {
                    g.leftBlobs = g.leftBlobs + 4;
                }
                else if (myBoxFitSize3.width/ myBoxFitSize3.height > 11.8 || myBoxFitSize3.width/ myBoxFitSize3.height < 0.09) {
                    g.leftBlobs = g.leftBlobs + 5;
                }
                else if (myBoxFitSize3.width/ myBoxFitSize3.height > 13.8 || myBoxFitSize3.width/ myBoxFitSize3.height < 0.078) {
                    g.leftBlobs = g.leftBlobs + 6;
                }
            }
            Size myBoxFitSize4;
            for(ColorBlobLocatorProcessor.Blob b : g.leftGreen)
            {
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize4 = boxFit.size;
                telemetry.addData("old green width", myBoxFitSize4.width);
                telemetry.addData("old green height", myBoxFitSize4.height);
                telemetry.addData("old green angle", boxFit.angle);
                if(myBoxFitSize4.width/ myBoxFitSize4.height > 3.8 || myBoxFitSize4.width/ myBoxFitSize4.height < 0.35 ){
                    g.leftBlobs = g.leftBlobs + 1;
                }
                else if (myBoxFitSize4.width/ myBoxFitSize4.height > 5.8 || myBoxFitSize4.width/ myBoxFitSize4.height < 0.18) {
                    g.leftBlobs = g.leftBlobs + 2;
                }
                else if (myBoxFitSize4.width/ myBoxFitSize4.height > 7.8 || myBoxFitSize4.width/ myBoxFitSize4.height < 0.135) {
                    g.leftBlobs = g.leftBlobs + 3;
                }
                else if (myBoxFitSize4.width/ myBoxFitSize4.height > 9.8 || myBoxFitSize4.width/ myBoxFitSize4.height < 0.11) {
                    g.leftBlobs = g.leftBlobs + 4;
                }
                else if (myBoxFitSize4.width/ myBoxFitSize4.height > 11.8 || myBoxFitSize4.width/ myBoxFitSize4.height < 0.09) {
                    g.leftBlobs = g.leftBlobs + 5;
                }
                else if (myBoxFitSize4.width/ myBoxFitSize4.height > 13.8 || myBoxFitSize4.width/ myBoxFitSize4.height < 0.078) {
                    g.leftBlobs = g.leftBlobs + 6;
                }
            }
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
                            switch(g.blobCount) {
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
                                    g.shoot(3);
                                    g.shoot(1);
                                    g.shoot(2);
                                    break;
                                case 3:
                                    g.shoot(1);
                                    g.shoot(2);
                                    g.shoot(3);
                                    break;
                            }
                            break;
                        case 2:
                            switch(g.blobCount) {
                                case 0:
                                    g.shoot(2);
                                    g.shoot(3);
                                    g.shoot(1);
                                    break;
                                case 1:
                                    g.shoot(3);
                                    g.shoot(1);
                                    g.shoot(2);
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
                                case 4:
                                    g.shoot(3);
                                    g.shoot(1);
                                    g.shoot(2);
                                    break;
                                case 5:
                                    g.shoot(1);
                                    g.shoot(2);
                                    g.shoot(3);
                                    break;
                                case 6:
                                    g.shoot(2);
                                    g.shoot(3);
                                    g.shoot(1);
                                    break;
                            }
                            break;
                        case 3:
                            switch(g.blobCount) {
                                case 0:
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
                                    break;
                                case 1:
                                    g.shoot(2);
                                    g.shoot(1);
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
                                    g.shoot(2);
                                    g.shoot(1);
                                    g.shoot(3);
                                    break;
                                case 5:
                                    g.shoot(1);
                                    g.shoot(3);
                                    g.shoot(2);
                                    break;
                                case 6:
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
                                    break;
                                case 7:
                                    g.shoot(2);
                                    g.shoot(1);
                                    g.shoot(3);
                                    break;
                                case 8:
                                    g.shoot(1);
                                    g.shoot(3);
                                    g.shoot(2);
                                    break;
                                case 9:
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
                                    break;
                            }
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
                            switch(g.blobCount) {
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
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
                                    break;
                                case 3:
                                    g.shoot(2);
                                    g.shoot(1);
                                    g.shoot(3);
                                    break;
                            }
                            break;
                        case 2:
                            switch(g.blobCount) {
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
                                    g.shoot(3);
                                    g.shoot(1);
                                    g.shoot(2);
                                    break;
                                case 3:
                                    g.shoot(1);
                                    g.shoot(2);
                                    g.shoot(3);
                                    break;
                                case 4:
                                    g.shoot(2);
                                    g.shoot(1);
                                    g.shoot(3);
                                    break;
                                case 5:
                                    g.shoot(3);
                                    g.shoot(1);
                                    g.shoot(2);
                                    break;
                                case 6:
                                    g.shoot(1);
                                    g.shoot(2);
                                    g.shoot(3);
                                    break;
                            }
                            break;
                        case 3:
                            switch(g.blobCount) {
                                case 0:
                                    //pgp
                                    g.shoot(2);//purple
                                    g.shoot(3);//green
                                    g.shoot(1);//purple
                                    break;
                                case 1:
                                    //gpp
                                    g.shoot(3);//green
                                    g.shoot(2);//purple
                                    g.shoot(1);//purple
                                    break;
                                case 2:
                                    //ppg
                                    g.shoot(2);//purple
                                    g.shoot(1);//purple
                                    g.shoot(3);//green
                                    break;
                                case 3:
                                    //pgp
                                    g.shoot(2);//purple
                                    g.shoot(3);//green
                                    g.shoot(1);//purple
                                    break;
                                case 4:
                                    //gpp
                                    g.shoot(3);//green
                                    g.shoot(2);//purple
                                    g.shoot(1);//purple
                                    break;
                                case 5:
                                    //ppg
                                    g.shoot(2);//purple
                                    g.shoot(1);//purple
                                    g.shoot(3);//green
                                    break;
                                case 6:
                                    //pgp
                                    g.shoot(2);//purple
                                    g.shoot(3);//green
                                    g.shoot(1);//purple
                                    break;
                                case 7:
                                    //gpp
                                    g.shoot(3);//green
                                    g.shoot(2);//purple
                                    g.shoot(1);//purple
                                    break;
                                case 8:
                                    //ppg
                                    g.shoot(2);//purple
                                    g.shoot(1);//purple
                                    g.shoot(3);//green
                                    break;
                                case 9:
                                    //pgp
                                    g.shoot(2);//purple
                                    g.shoot(3);//green
                                    g.shoot(1);//purple
                                    break;
                            }
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
                            switch(g.blobCount) {
                                case 0:
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
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
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
                                    break;
                            }
                            break;
                        case 2:
                            switch(g.blobCount) {
                                case 0:
                                    g.shoot(3);
                                    g.shoot(1);
                                    g.shoot(2);
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
                                    g.shoot(1);
                                    g.shoot(2);
                                    break;
                                case 4:
                                    g.shoot(1);
                                    g.shoot(2);
                                    g.shoot(3);
                                    break;
                                case 5:
                                    g.shoot(2);
                                    g.shoot(1);
                                    g.shoot(3);
                                    break;
                                case 6:
                                    g.shoot(3);
                                    g.shoot(1);
                                    g.shoot(2);
                                    break;
                            }
                            break;
                        case 3:
                            switch(g.blobCount) {
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
                                    g.shoot(3);
                                    g.shoot(2);
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
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
                                    break;
                                case 6:
                                    g.shoot(1);
                                    g.shoot(2);
                                    g.shoot(3);
                                    break;
                                case 7:
                                    g.shoot(2);
                                    g.shoot(3);
                                    g.shoot(1);
                                    break;
                                case 8:
                                    g.shoot(3);
                                    g.shoot(2);
                                    g.shoot(1);
                                    break;
                                case 9:
                                    g.shoot(1);
                                    g.shoot(2);
                                    g.shoot(3);
                                    break;
                            }
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
    public static Pose launchPose = new Pose(144-launchPoseX, launchPoseY, Math.toRadians(110));
    public static Pose aprilTagPose = BlueAuto.aprilTagPose.mirror().setHeading(Math.toRadians(200));

    public static Pose gotoFirstCtrl = mirrorNoHeading(BlueAuto.gotoFirstCtrl);
    public static Pose firstIntakePose = mirrorNoHeading(BlueAuto.firstIntakePose);
    public static Pose firstIntakeEnd = mirrorNoHeading(BlueAuto.firstIntakeEnd);

    public static Pose secondGotoPose = mirrorNoHeading(BlueAuto.secondGotoPose);
    public static Pose secondIntakePose = mirrorNoHeading(BlueAuto.secondIntakePose);

    public static Pose thirdGotoPose = mirrorNoHeading(BlueAuto.thirdGotoPose);
    public static Pose thirdIntakePose = mirrorNoHeading(BlueAuto.thirdIntakePose);

    public static Pose parkPose = mirrorNoHeading(BlueAuto.parkPose).setHeading(launchPose.getHeading());

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        g = new GreenFN(hardwareMap);
        GreenFN.minTimeBtwnShots = 0.7;
        g.loadStoppers();
        g.follower.setStartingPose(startPose);

        Path aprilTag =
                new Path(
                        new BezierLine(startPose, aprilTagPose)
                );
        aprilTag.setLinearHeadingInterpolation(startPose.getHeading(), aprilTagPose.getHeading());

        Path scorePreload =
                new Path(
                        new BezierLine(aprilTagPose, launchPose)
                );
        scorePreload.setLinearHeadingInterpolation(aprilTagPose.getHeading(), launchPose.getHeading()+Math.toRadians(11));

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
        scoreThird.setLinearHeadingInterpolation(Math.toRadians(0), launchPose.getHeading()+Math.toRadians(1.5));

        Path park =
                new Path(
                        new BezierLine(launchPose, parkPose)
                );
        park.setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(183));


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
                                g.setShooterRPM((int) GreenFN.shooterAutoRPM-21),
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
                                new WaitToDone(),
                                new InstantAction(() -> GreenFN.detect=true),
                                follow(scorePreload),
                                g.loadAction(),
                                new WaitToDone(),
                                g.setIntakeRPM(GreenFN.intakeRPM + 250),
                                new SleepAction(1),
                                shootInOrder( 0),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                g.loadAction(),

                                followUntilDone(gotoFirst),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+250),
                                new InstantAction(() -> g.follower.setMaxPower(0.7)),
                                followUntilDone(intakeFirst),
                                new InstantAction(() -> g.follower.setMaxPower(1)),
                                new SleepAction(0.2),
                                follow(scoreFirst),
                                new WaitToDone(),
//                                new SleepAction(0.5),
                                shootInOrder(1),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                g.loadAction(),
                                followUntilDone(gotoSecond),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+250),
                                new InstantAction(() -> g.follower.setMaxPower(0.7)),
                                followUntilDone(intakeSecond),
                                new InstantAction(() -> g.follower.setMaxPower(1)),
                                new SleepAction(0.2),
                                follow(scoreSecond),
                                new WaitToDone(),
//                                new SleepAction(0.5),
                                shootInOrder(2),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                g.loadAction(),
                                followUntilDone(gotoThird),
                                g.setIntakeRPM((int) GreenFN.intakeRPM+250),
                                new InstantAction(() -> g.follower.setMaxPower(0.7)),
                                followUntilDone(intakeThird),
                                new InstantAction(() -> g.follower.setMaxPower(1)),
                                new SleepAction(0.2),
                                follow(scoreThird),
                                new WaitToDone(),
//                                new SleepAction(0.5),
                                shootInOrder(3),
                                new SleepAction(GreenFN.minTimeBtwnShots *3),
                                followUntilDone(park)
                        )
                )
        );

    }

}
