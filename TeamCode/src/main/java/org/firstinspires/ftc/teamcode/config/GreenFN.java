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
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;




@Config
public class GreenFN {
    public static double shooterCloseRPM = 880;
    public static double intakeRPM = 300;
    public static double closeHoodAngle = 80;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    PIDController shooterController1, shooterController2, intakeController;
    public static double intakekP = 0.01, intakekI=0, intakekD=0.00, shooterkP=0.01, shooterkI=0.1, shooterkD=0.0001, intakeKF = 0.000285, shooterkF = 0.0002;
    public DcMotorEx shooter1, shooter2, intake;
    public ColorSensor colorSensor1, colorSensor2, colorSensor3;
    RevBlinkinLedDriver lights;
    public static double shooterTarget, intakeTarget;
    public static double actualShooterCloseRPM = 810;

    public Servo stopper1, stopper2, stopper3;
    public Follower follower;

    public double minTimeBtwnShots = 0.4;


    public static int automotif = -1;

    public static double intakePower, flywheel1Power, flywheel2Power;


    public static double stopperIntake=0.83, stopperLoad=0.61, stopperShoot=0.13;
    public static double stopper1Load = 0.65, stopper2Load = 0.47, stopper3Load = 0.55;
    public static double stopper1Intake = 0.82, stopper2Intake = 0.64, stopper3Intake = 0.74;

    public ArrayList<Servo> shooterQueue = new ArrayList<>();
    public int shooterRPMTolerance = 70;

    public enum Motifs {
        GPP,
        PGP,
        PPG
    }

    public static Motifs currentMotif = null;

    public Timer shotTimer;
    public double recentShotTime = 0;

    // 🔧 Camera position offsets relative to robot center (in inches)
    public static final float CAMERA_X_OFFSET = 3.5f;
    public static final float CAMERA_Y_OFFSET = 7.0f;
    public static final float CAMERA_Z_OFFSET = 4.0f;

    // 🗺️ Define AprilTag field positions (example values for Centerstage)
    // Tag IDs and their coordinates on the field
    public static final TagPose TAG_1 = new TagPose(0f, 0f, 0f);
    public static final TagPose TAG_2 = new TagPose(60f, 36f, 0f);
    public static final TagPose TAG_3 = new TagPose(120f, 36f, 180f);
    public static final TagPose TAG_4 = new TagPose(60f, 108f, 0f);
    // 🔹 Add or edit these with the real field coordinates for your game

    public static class TagPose {
        public float x, y, heading;
        public TagPose(float x, float y, float heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
    // --- Method to get a tag's known field position ---
    public static TagPose getTagFieldPosition(int id) {
        switch (id) {
            case 1: return TAG_1;
            case 2: return TAG_2;
            case 3: return TAG_3;
            case 4: return TAG_4;
            default: return null;
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
                follower.followPath(a, holdEnd);
                called = true;
            }
            return follower.isBusy();
        }
    }

    public class FollowUntilDoneChain implements Action {

        PathChain a;
        boolean called = false;

        public FollowUntilDoneChain(PathChain a)  {
            this.a = a;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!called) {
                follower.followPath(a);
                called = true;
            }
            return follower.isBusy();
        }
    }

    public GreenFN(HardwareMap map) {
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

//        colorSensor1 = map.get(ColorSensor.class, "colorSensor1");
//        colorSensor2 = map.get(ColorSensor.class, "colorSensor2");
//        colorSensor3 = map.get(ColorSensor.class, "colorSensor3");
//        lights = map.get(RevBlinkinLedDriver.class, "lights");
//        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        // Create the AprilTag processor the easy way.
        follower = Constants.createFollower(map);
        follower.setStartingPose(new Pose(0,0,0));
        follower.setPose(new Pose(0,0,0));
        // Initialize camera (webcam example)
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                map.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
    }

    /**
     * Add telemetry about AprilTag detections.
     */

    public void update() {
        updatePIDs();
        follower.update();
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.size() > 0) {
            AprilTagDetection detection = detections.get(0); // pick the first detected tag

            // Convert to field coordinates
            float[] robotPos = convertToFieldPosition(detection);
        }



        List<Servo> toRemove = new ArrayList<>();
        for (Servo s : shooterQueue) {
            if (shotTimer.getElapsedTimeSeconds() - recentShotTime > minTimeBtwnShots) {
                s.setPosition(stopperShoot);
                toRemove.add(s);
                recentShotTime = shotTimer.getElapsedTimeSeconds();
            }
        }
        shooterQueue.removeAll(toRemove);


        if (currentMotif == null) {
            updateAprilTagDetection();
        }
    }

    public void intakeStoppers() {
        stopper1.setPosition(stopper1Intake);
        stopper2.setPosition(stopper2Intake);
        stopper3.setPosition(stopper3Intake);
    }

    public void loadStoppers() {
        stopper1.setPosition(stopper1Load);
        stopper2.setPosition(stopper2Load);
        stopper3.setPosition(stopper3Load);
    }

    public void shoot(int shooter) {
        switch(shooter) {
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

    public void load() {
        stopper1.setPosition(stopper1Load);
        stopper2.setPosition(stopper2Load);
        stopper3.setPosition(stopper3Load);
    }

    public void updatePIDs() {
        flywheel1Power = shooterController1.calculate(shooter1.getVelocity(AngleUnit.DEGREES), shooterTarget) + shooter1.getVelocity(AngleUnit.DEGREES)* shooterkF;
        flywheel2Power = shooterController2.calculate(shooter2.getVelocity(AngleUnit.DEGREES), shooterTarget) + shooter2.getVelocity(AngleUnit.DEGREES)* shooterkF;

        intakePower = intakeController.calculate(intake.getVelocity(AngleUnit.DEGREES), intakeTarget) + intake.getVelocity(AngleUnit.DEGREES)*intakeKF;

        shooter1.setPower(flywheel1Power);
        shooter2.setPower(flywheel1Power);
        intake.setPower(intakePower);

        intakeController.setPID(intakekP, intakekI, intakekD);
        shooterController1.setPID(shooterkP, shooterkI, shooterkD);
        shooterController1.setPID(shooterkP, shooterkI, shooterkD);
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

    public Action setIntakeRPM(int rpm) {
        return new InstantAction(() -> {intakeTarget=rpm;});
    }

    public Action setShooterRPM(int rpm) { return new InstantAction(() -> shooterTarget =rpm); }

    public Action shootAction(int shooter) {return new InstantAction(() -> shoot(shooter));}
    public Action loadAction() {return new InstantAction(this::load);}
    /**
     * Converts an AprilTag detection into the robot’s absolute field position.
     * @param detection The AprilTag detection from AprilTagProcessor
     * @return float[] {robotX, robotY, robotHeading} in field coordinates
     */
    public static float[] convertToFieldPosition(AprilTagDetection detection) {
        if (detection == null) {
            return new float[]{Float.NaN, Float.NaN, Float.NaN};
        }

        TagPose tagFieldPosition = getTagFieldPosition(detection.id);
        if (tagFieldPosition == null) {
            return new float[]{Float.NaN, Float.NaN, Float.NaN};
        }

        float tagX   = (float) detection.ftcPose.x;
        float tagY   = (float) detection.ftcPose.y;
        float tagYaw = (float) detection.ftcPose.yaw;

        float robotX_fromTag = -tagX - CAMERA_X_OFFSET;
        float robotY_fromTag = -tagY - CAMERA_Y_OFFSET;

        float robotFieldX = tagFieldPosition.x + robotX_fromTag;
        float robotFieldY = tagFieldPosition.y + robotY_fromTag;
        float robotHeading = tagFieldPosition.heading + tagYaw;

        return new float[]{robotFieldX, robotFieldY, robotHeading};
    }


}

/*public void loop() {
  if(headingLock) {
    double targetHeading = Math.toRadians(180);

    double headingError = targetHeading - currentHeading;
    headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

    if (Math.abs(headingError) < Math.toRadians(2)) {
      headingCorrection = 0;
    } else {
      headingCorrection = headingPIDController.calculate(headingError);
    }

    follower.setTeleOpMovementVectors(-gamepad1.getLeftY(), gamepad1.getLeftX(), headingCorrection);

  } else {
    follower.setTeleOpMovementVectors(-gamepad1.getLeftY(), gamepad1.getLeftX(), gamepad1.getRightX());
  }
}*/