package org.firstinspires.ftc.teamcode.config;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;




@Config
public class GreenFN {
    public static double shooterCloseRPM = 645;
    public static double shooterAutoRPM = 668;
    public static double intakeRPM = 600;
    public static double closeHoodAngle = 80;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    enum Colors {
        PURPLE,
        GREEN,
        NONE
    }

    public RevColorSensorV3 colorSensor1, colorSensor2, colorSensor3;

    PIDController shooterController1, shooterController2, intakeController;
    public static double intakekP = 0.002, intakekI = 0, intakekD = 0.00, shooterkP = 0.05, shooterkI = 0.1, shooterkD = 0.0001, intakeKF = 0.0009, shooterkF = 0.00024;
    public DcMotorEx shooter1, shooter2, intake;
//    public NormalizedColorSensor colorSensor1, colorSensor2, colorSensor3;
    RevBlinkinLedDriver lights;
    public static double shooterTarget = 0, intakeTarget = 0;
    public static double actualShooterCloseRPM = 810;
    public static double shooterFarRPM = 1100;

    public static double[] distances = new double[3];
    public static NormalizedRGBA[] colors = new NormalizedRGBA[3];

    public static boolean[] ballInSlot = new boolean[]{false, false, false};
    public static Colors[] ballColors = new Colors[]{Colors.NONE, Colors.NONE, Colors.NONE};

    public boolean colorSensorUpdated = false, slotsUpdated=false;
    public boolean toUpdateColorSensor = false, toUpdateSlots = false;
    public int recentServoShot = 0;

    public double s1v = 0, s2v = 0, iv = 0;

    public double adjustedX = 0, adjustedY = 0;


    public Servo stopper1, stopper2, stopper3;
    public Follower follower;

    public static double minTimeBtwnShots = 0.52;

    public boolean loaded = true, divided = false;


    public static int automotif = -1;

    public static double intakePower, flywheel1Power, flywheel2Power;
    public Pose currentPose = new Pose(0, 0, 0);
    public double angvel = 0;

    // --- Heading Lock Variables ---
    private boolean headingLockEnabled = false;
    private double lockedHeading = 0;
    private PIDController headingLockController = new PIDController(0.02, 0, 0.001);


    public static double stopperIntake = 0.83, stopperLoad = 0.61, stopperShoot = 0.05;
    public static double stopper1Shoot = 0.025, stopper2Shoot = 0.05, stopper3Shoot = 0.1;
    public static double stopper1Load = 0.32, stopper2Load = 0.34; //0.47
    public static double stopper3Load = 0.39;
    public static double stopper1Intake = 0.54, stopper2Intake = 0.59;// 0.655,
    public static double stopper3Intake = 0.62;

    public double stop1pos, stop2pos, stop3pos;

    public ArrayList<Servo> shooterQueue = new ArrayList<>();
    public int shooterRPMTolerance = 70;

    FirstBoolean lodded = new FirstBoolean();


    public enum Motifs {
        GPP,
        PGP,
        PPG
    }

    public static Motifs currentMotif = null;

    public int counter = 0;
    public int colorCounter = 0;
    public static boolean detect = true;

    public Timer shotTimer;
    public double recentShotTime = 0;
    public static Pose fromApriltag = new Pose(0, 0, 0);
    public static double modifiedShooterRPM = shooterTarget;

    // 🔧 Camera position offsets relative to robot center (in inches)
    public static final double CAMERA_X_OFFSET = 2.2;
    public static final double CAMERA_Y_OFFSET = 5.6f;
    public static final double CAMERA_Z_OFFSET = 4.0f;
    public static final double CAMERA_TILT_DEGREES = 20.0f; // degrees tilted back (affects distance slightly)

    // 🗺️ Define AprilTag field positions (example values for Centerstage)
    // Tag IDs and their coordinates on the field
    public static Pose TAG_20 = new Pose(15.5, 131.5, Math.toRadians(-36));
    public static final Pose TAG_24 = new Pose(128.6, 131.5, Math.toRadians(216));
    public static Pose TAG_20_SHIFTED = new Pose(127.7, 13, Math.toRadians(180-36));
    //public static final TagPose TAG_3 = new TagPose(120f, 36f, 180f);
    //public static final TagPose TAG_4 = new TagPose(60f, 108f, 0f);
    // 🔹 Add or edit these with the real field coordinates for your game


    // --- Method to get a tag's known field position ---
    public static Pose getTagFieldPosition(int id) {
        switch (id) {
            case 20:
                return TAG_20;
            case 24:
                return TAG_24;
            default:
                return null;
        }
    }


    public class WaitToDone implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return follower.isBusy();
        }
    }

    public Action follow(PathChain a) {
        return new InstantAction(() -> {
            if (!follower.isBusy()) {
                follower.followPath(a, true);
            }
        });
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

    public Action followPathJustPath(Path a) {
        return new InstantAction(() -> follower.followPath(a));
    }

    public Motifs getCurrentMotif() {
        return currentMotif;
    }

    public class FollowUntilDone implements Action {

        Path a;
        boolean called = false;
        boolean holdEnd = true;

        public FollowUntilDone(Path a) {
            this.a = a;
        }

        public FollowUntilDone(Path a, boolean holdEnd) {
            this.a = a;
            this.holdEnd = holdEnd;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!called) {
                follower.followPath(a, holdEnd);
                called = true;
            }
            return follower.isBusy();
        }
    }

    public class FollowUntilDoneChain implements Action {

        PathChain a;
        boolean called = false;

        public FollowUntilDoneChain(PathChain a) {
            this.a = a;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!called) {
                follower.followPath(a);
                called = true;
            }
            return follower.isBusy();
        }
    }

    public GreenFN(HardwareMap map) {
        List<LynxModule> allHubs = map.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        shooter1 = map.get(DcMotorEx.class, "shooter1");
        shooter2 = map.get(DcMotorEx.class, "shooter2");
        intake = map.get(DcMotorEx.class, "intake");
        shooterController1 = new PIDController(shooterkP, shooterkI, shooterkD);
        shooterController2 = new PIDController(shooterkP, shooterkI, shooterkD);
        intakeController = new PIDController(intakekP, intakekI, intakekD);
        stopper1 = map.servo.get("stopper1");
        stopper2 = map.servo.get("stopper2");
        stopper3 = map.servo.get("stopper3");


        shotTimer = new Timer();
        recentShotTime = shotTimer.getElapsedTimeSeconds();

        colorSensor1 = map.get(RevColorSensorV3.class, "colorSensor1");
        colorSensor2 = map.get(RevColorSensorV3.class, "colorSensor2");
        colorSensor3 = map.get(RevColorSensorV3.class, "colorSensor3");
//        lights = map.get(RevBlinkinLedDriver.class, "lights");
//        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        // Create the AprilTag processor the easy way.
        follower = Constants.createFollower(map);
        // Initialize camera (webcam example)
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                map.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
    }

    /**
     * Add telemetry about AprilTag detections.
     */

    public void update() {

        updatePIDs();
        currentPose = follower.getPose();
        angvel = follower.poseTracker.getAngularVelocity();

        follower.update();

        iv = intake.getVelocity(AngleUnit.DEGREES);
        s1v = shooter1.getVelocity(AngleUnit.DEGREES);
        s2v = shooter2.getVelocity(AngleUnit.DEGREES);
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//        if (detections.size() > 0) {
//            AprilTagDetection detection = detections.get(0); // pick the first detected tag
//
//            // Convert to field coordinates
//            if(detection.id == 20 || detection.id == 24) {
//                fromApriltag = convertToFieldPosition(detection);
//            }
//        }


        if(counter%20==0) {
            stop1pos = stopper1.getPosition();
            stop2pos = stopper2.getPosition();
            stop3pos = stopper3.getPosition();
            stopper1.setPosition(stop1pos);
            stopper2.setPosition(stop2pos);
            stopper3.setPosition(stop3pos);
            counter = 0;
        }
        counter++;
//        loadStoppers();

        List<Servo> toRemove = new ArrayList<>();
        for (Servo s : shooterQueue) {
            if (shotTimer.getElapsedTimeSeconds() - recentShotTime > minTimeBtwnShots) {
                toRemove.add(s);
                if (s.equals(stopper1)) {
                    recentServoShot = 1;
                    stopper1.setPosition(stopper1Shoot);
                }
                if (s.equals(stopper2)) {
                    recentServoShot = 2;
                    stopper2.setPosition(stopper2Shoot);
                }
                if (s.equals(stopper3)) {
                    recentServoShot = 3;
                    stopper3.setPosition(stopper3Shoot);
                }
                recentShotTime = shotTimer.getElapsedTimeSeconds();
            }
        }
        shooterQueue.removeAll(toRemove);


        if (!loaded) {
            modifiedShooterRPM = 0;
        }
        if (loaded) {
            modifiedShooterRPM = shooterTarget;
        }

        if (currentMotif == null && detect) {
            updateAprilTagDetection();

        }
        colorSensorUpdated = false;
        if(toUpdateColorSensor) {
            updateColorSensor();
        }
        slotsUpdated = false;
        if(toUpdateSlots) {
            updateSlotDetection();
        }
    }

    public Pose getPose() {
        return currentPose;
    }

    public void intakeStoppers() {
        stopper1.setPosition(stopper1Intake-0.2);
        stopper2.setPosition(stopper2Intake-0.2);
        stopper3.setPosition(stopper3Intake-0.2);
        stopper1.setPosition(stopper1Intake);
        stopper2.setPosition(stopper2Intake);
        stopper3.setPosition(stopper3Intake);
        loaded = false;
    }

    public void loadStoppers() {
        stopper1.setPosition(stopper1Load+0.2);
        stopper2.setPosition(stopper2Load+0.2);
        stopper3.setPosition(stopper3Load+0.2);
        stopper1.setPosition(stopper1Load);
        stopper2.setPosition(stopper2Load);
        stopper3.setPosition(stopper3Load);
        loaded = true;
    }

    public void shoot(int shooter) {
        switch (shooter) {
            case 1:
                shooterQueue.add(stopper1);
//                stopper1.setPosition(stopperShoot);
                break;
            case 2:
                shooterQueue.add(stopper2);
//                stopper2.setPosition(stopperShoot);
                break;
            case 3:
                shooterQueue.add(stopper3);
//                stopper3.setPosition(stopperShoot);
                break;
            case 4:
                shooterQueue.add(stopper1);
                shooterQueue.add(stopper2);
                shooterQueue.add(stopper3);
//                stopper1.setPosition(stopperShoot);
//                stopper2.setPosition(stopperShoot);
//                stopper3.setPosition(stopperShoot);

                break;
        }
    }

    public void updateSlotDetection() {
        if(colorCounter%5==0) {
            updateColorSensor();
            colorCounter=0;
        }
        for(int i = 0; i < 3; i++) {
            if(distances[i] < 1.3) {
                ballInSlot[i] = true;
                if(colors[i].green>0.03) {
                    ballColors[i] = Colors.GREEN;
                }
                else {
                    ballColors[i] = Colors.PURPLE;
                }
            }
            else {
                ballInSlot[i] = false;
                ballColors[i] = Colors.NONE;
            }
        }
        slotsUpdated = true;
    }

    public void load() {
        stopper1.setPosition(stopper1Load);
        stopper2.setPosition(stopper2Load);
        stopper3.setPosition(stopper3Load);
        loaded = true;
    }

    public void updatePIDs() {
        flywheel1Power = shooterController1.calculate(s1v, modifiedShooterRPM) + s1v * shooterkF;
        flywheel2Power = shooterController2.calculate(s2v, modifiedShooterRPM) + s2v * shooterkF;

        intakePower = intakeController.calculate(iv, intakeTarget) + iv * intakeKF;
        intakeController.setPID(intakekP, intakekI, intakekD);
        shooterController1.setPID(shooterkP, shooterkI, shooterkD);
        shooterController1.setPID(shooterkP, shooterkI, shooterkD);
        intake.setPower(intakePower);

        if(modifiedShooterRPM==0) {
            shooter1.setPower(0);
            shooter2.setPower(0);
            return;
        }

        shooter1.setPower(flywheel1Power);
        shooter2.setPower(flywheel1Power);



    }

    public void updateColorSensor() {
        distances[0] = colorSensor1.getDistance(DistanceUnit.INCH);
        distances[1] = colorSensor2.getDistance(DistanceUnit.INCH);
        distances[2] = colorSensor3.getDistance(DistanceUnit.INCH);
        colors[0] = colorSensor1.getNormalizedColors();
        colors[1] = colorSensor2.getNormalizedColors();
        colors[2] = colorSensor3.getNormalizedColors();
        colorSensorUpdated = true;
    }

//    public void updateLights() {
//        int red1 = colorSensor1.red();
//        int green1 = colorSensor1.green();
//        int blue1 = colorSensor1.blue();
//
//        int red2 = colorSensor2.red();
//        int green2 = colorSensor2.green();
//        int blue2 = colorSensor2.blue();
//
//        int red3 = colorSensor3.red();
//        int green3 = colorSensor3.green();
//        int blue3 = colorSensor3.blue();
//
//        if ((red1 > 150 && blue1 > 150 && green1 < 100) && (red2 < 50 && blue2 < 100 && green2 > 175) && (red3 > 150 && blue3 > 150 && green3 < 100)) {
//            // Likely purple green purple
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//        }
//        else if ((red1 > 150 && blue1 > 150 && green1 < 100) && (red2 > 150 && blue2 > 150 && green2 < 100) &&(green3 > 200 && red3 < 100 && blue3 < 100)) {
//            // Likely purple purple green                        needs to be changed
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
//        }
//        else if ((red1 < 100 && blue1 < 100 && green1 > 175) && (red2 > 150 && blue2 > 150 && green2 < 100) && (red3 > 150 && blue3 > 150 && green3 < 100)){
//            // Likely green purple purple
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
//        }
//        else {
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
//        }
//    }

    public List<AprilTagDetection> getApriltagDetections() {
        return aprilTag.getDetections();
    }

    public double getGoalHeadingTarget() {
        List<AprilTagDetection> detections = getApriltagDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 24 || detection.id == 20) {
                return getPose().getHeading() + detection.ftcPose.bearing;
            }
        }
        return getPose().getHeading();
    }

    public void updateAprilTagDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            switch (detection.id) {
                case 21:
                    currentMotif = Motifs.GPP;
                    break;
                case 22:
                    currentMotif = Motifs.PGP;
                    break;
                case 23:
                    currentMotif = Motifs.PPG;
                    break;
            }
        }
    }

    public Action updateAction() {
        return new InstantAction(this::update);
    }

    public Action setIntakeRPM(double rpm) {
        return new InstantAction(() -> {
            intakeTarget = rpm;
        });
    }

    public Action setShooterRPM(double rpm) {
        return new InstantAction(() -> shooterTarget = rpm);
    }

    public Action shootAction(int shooter) {
        return new InstantAction(() -> shoot(shooter));
    }

    public Action loadAction() {
        return new InstantAction(this::load);
    }

    /**
     * Converts an AprilTag detection into the robot’s absolute field position.
     *
     * @param detection The AprilTag detection from AprilTagProcessor
     * @return double[] {robotX, robotY, robotHeading} in field coordinates
     */
    public static Pose convertToFieldPosition(AprilTagDetection detection) {
        if (detection == null) {
            return new Pose(0, 0, 0);
        }

        Pose tagFieldPosition = getTagFieldPosition(detection.id);
        if (tagFieldPosition == null) {
            return new Pose(0, 0, 0);
        }

        double tagX = (double) detection.ftcPose.x;
        double tagY = (double) detection.ftcPose.y;
        double tagYaw = (double) detection.ftcPose.yaw;
        double correctionX = tagYaw;


        // Optional: adjust distance due to tilt (simple correction)
        double tiltCorrection = (double) Math.cos(Math.toRadians(tagYaw));
        tagX *= Math.abs(tiltCorrection);

        double robotX_fromTag = -tagX - CAMERA_X_OFFSET;
        double robotY_fromTag = -tagY - CAMERA_Y_OFFSET;

        double robotFieldX = tagFieldPosition.getX() + robotX_fromTag;
        double robotFieldY = tagFieldPosition.getY() + robotY_fromTag;
        double robotHeading = tagFieldPosition.getHeading() + tagYaw;

        fromApriltag = new Pose(robotFieldX, robotFieldY, robotHeading);

        return fromApriltag;
    }

    public static double getRPMreg(double range) {
        return 0.0692362*range*range-3.38259*range+718.12462;
    }

    public static Pose convertToFieldPosition2(AprilTagDetection detection) {
        if (detection == null) {
            return new Pose(0, 0, 0);
        }

        Pose tagFieldPosition = getTagFieldPosition(detection.id);
        if (tagFieldPosition == null) {
            return new Pose(0, 0, 0);
        }

        double tagX = (double) detection.ftcPose.x;
        double tagY = (double) detection.ftcPose.y;
        double tagYaw = (double) detection.ftcPose.yaw;
        double correctionX = tagYaw;


        double x = detection.ftcPose.range * Math.sin(tagYaw);
        double y = detection.ftcPose.range * Math.cos(tagYaw);


//        Pose fromTag =

        double tagHeading = tagFieldPosition.getHeading();

//        double robotX = tagFieldPosition.getX() + x*Math.sin

//        fromApriltag = new Pose(robotFieldX, robotFieldY, robotHeading);

        return fromApriltag;
    }

    // Call this in TeleOp when you want to toggle the heading lock
    public void toggleHeadingLock(Pose currentPose) {
        headingLockEnabled = !headingLockEnabled;
        if (headingLockEnabled) {
            lockedHeading = currentPose.getHeading(); // Lock to current angle
        }
    }

    // Call this every loop if heading lock is ON
    public double getHeadingCorrection(Pose currentPose) {
        if (!headingLockEnabled) return 0;

        double error = lockedHeading - currentPose.getHeading();
        error = Math.toRadians(AngleUnit.normalizeDegrees(Math.toDegrees(error)));
        return headingLockController.calculate(0, error);
    }

    public boolean isHeadingLockEnabled() {
        return headingLockEnabled;
    }

    // --- AUTO-AIM USING APRILTAG ---
// This method calculates a correction to make the robot face a specific AprilTag (e.g., the goal)
    // Returns the Pose of the visible goal tag (20 or 24), or null if none
    public AprilTagDetection getVisibleGoalPose() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection det : detections) {
            if (det.id == 20 || det.id == 24) {
                return det;
            }
        }
        return null;
    }

    public Pose getPoseFromApriltag() {
        AprilTagDetection goal = getVisibleGoalPose();
        if(goal==null) {
            return new Pose(getPose().getX()+adjustedX, getPose().getY()+adjustedY, getPose().getHeading());
        }
        double range = goal.ftcPose.range*Math.cos(Math.toRadians(goal.ftcPose.pitch));
        double headingThing = getPose().getHeading()+Math.toRadians(goal.ftcPose.bearing)+Math.toRadians(90);
        double x = range*Math.cos(headingThing);
        double y = range*Math.sin(headingThing);
        double newX=0, newY=0;
        double currentHeading = getPose().getHeading();
        if(getPose().getHeading()>1.5) {
            if (goal.id==24) {
                newX = TAG_24.getX()+x;
                newY = TAG_24.getY()+y;
            }
        }
        else {
            if(goal.id==20) {
                newX = TAG_20_SHIFTED.getX()+x-CAMERA_X_OFFSET;
                newY = TAG_20_SHIFTED.getY()+y-CAMERA_Y_OFFSET;
            }
        }
        if(newX!=0) {
            adjustedX = newX - getPose().getX();
            adjustedY = newY - getPose().getX();
        }
        return new Pose(getPose().getX()+adjustedX, getPose().getY()+adjustedY, getPose().getHeading());
    }
}
