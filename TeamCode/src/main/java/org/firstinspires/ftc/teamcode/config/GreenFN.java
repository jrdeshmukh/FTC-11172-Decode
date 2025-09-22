package org.firstinspires.ftc.teamcode.config;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;



@Config
public class GreenFN {
    public static double shooterCloseRPM = 4000;
    public static double closeHoodAngle = 80;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    PIDController shooterController1, shooterController2, intakeController;
    public static double intakekP = 0.00017, intakekI, intakekD, shooterkP, shooterkI, shooterkD, intakeKF = 0.000285, shooterkF = 0.000285;
    public DcMotorEx shooter1, shooter2, intake;
    public ColorSensor colorSensor1, colorSensor2, colorSensor3;
    RevBlinkinLedDriver lights;
    public static double shooterTarget, intakeTarget;

    public Servo stopper1, stopper2, stopper3;
    public Follower follower;

    public double minTimeBtwnShots = 0.2;


    public static int automotif = -1;

    public static double intakePower, flywheel1Power, flywheel2Power;


    public static double stopperIntake, stopperLoad, stopperShoot;

    public ArrayList<Servo> shooterQueue = new ArrayList<>();
    public int shooterRPMTolerance = 100;

    public enum Motifs {
        GPP,
        PGP,
        PPG
    }

    public static Motifs currentMotif = null;

    public Timer shotTimer;
    public double recentShotTime = 0;


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

//        colorSensor1 = map.get(ColorSensor.class, "colorSensor1");
//        colorSensor2 = map.get(ColorSensor.class, "colorSensor2");
//        colorSensor3 = map.get(ColorSensor.class, "colorSensor3");
//        lights = map.get(RevBlinkinLedDriver.class, "lights");
//        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        follower = Constants.createFollower(map);
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                map.get(WebcamName.class, "Webcam 1"), aprilTag);


    }

    /**
     * Add telemetry about AprilTag detections.
     */

    public void update() {
        updatePIDs();
        follower.update();



        for(Servo s: shooterQueue) {
            if(Math.abs(shooter1.getVelocity()/6.0-shooterTarget)<shooterRPMTolerance && shotTimer.getElapsedTimeSeconds()-recentShotTime>minTimeBtwnShots) {
                s.setPosition(stopperShoot);
                recentShotTime = shotTimer.getElapsedTimeSeconds();
            }
        }

        if (currentMotif == null) {
            updateAprilTagDetection();
        }
    }

    public void intakeStoppers() {
        stopper1.setPosition(stopperIntake);
        stopper2.setPosition(stopperIntake);
        stopper3.setPosition(stopperIntake);
    }

    public void loadStoppers() {
        stopper1.setPosition(stopperLoad);
        stopper2.setPosition(stopperLoad);
        stopper3.setPosition(stopperLoad);
    }

    public void shoot(int shooter) {
        switch(shooter) {
            case 1:
                shooterQueue.add(stopper1);
                break;
            case 2:
                shooterQueue.add(stopper2);
                break;
            case 3:
                shooterQueue.add(stopper3);
                break;
            case 4:
                shooterQueue.add(stopper1);
                shooterQueue.add(stopper2);
                shooterQueue.add(stopper3);
                break;
        }
    }

    public void load() {
        stopper1.setPosition(stopperLoad);
        stopper2.setPosition(stopperLoad);
        stopper3.setPosition(stopperLoad);
    }

    public void updatePIDs() {
        flywheel1Power = shooterController1.calculate(shooter1.getVelocity(AngleUnit.DEGREES)/6.0, shooterTarget) + shooter1.getVelocity()/6.0* shooterkF;
        flywheel2Power = shooterController2.calculate(shooter2.getVelocity(AngleUnit.DEGREES)/6.0, shooterTarget) + shooter2.getVelocity()/6.0* shooterkF;

        intakePower = intakeController.calculate(intake.getVelocity(AngleUnit.DEGREES)/6.0, intakeTarget) + intake.getVelocity()/6.0*intakeKF;

        shooter1.setPower(flywheel1Power);
        shooter2.setPower(flywheel1Power);
        intake.setPower(intakePower);

        intakeController.setPID(intakekP, intakekI, intakekD);
        shooterController1.setPID(shooterkP, shooterkI, shooterkD);
        shooterController1.setPID(shooterkP, shooterkI, shooterkD);
    }

    public void updateLights() {
        int red1 = colorSensor1.red();
        int green1 = colorSensor1.green();
        int blue1 = colorSensor1.blue();

        int red2 = colorSensor2.red();
        int green2 = colorSensor2.green();
        int blue2 = colorSensor2.blue();

        int red3 = colorSensor3.red();
        int green3 = colorSensor3.green();
        int blue3 = colorSensor3.blue();

        if ((red1 > 150 && blue1 > 150 && green1 < 100) && (red2 < 50 && blue2 < 100 && green2 > 175) && (red3 > 150 && blue3 > 150 && green3 < 100)) {
            // Likely purple green purple
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
        else if ((red1 > 150 && blue1 > 150 && green1 < 100) && (red2 > 150 && blue2 > 150 && green2 < 100) &&(green3 > 200 && red3 < 100 && blue3 < 100)) {
            // Likely purple purple green                        needs to be changed
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
        }
        else if ((red1 < 100 && blue1 < 100 && green1 > 175) && (red2 > 150 && blue2 > 150 && green2 < 100) && (red3 > 150 && blue3 > 150 && green3 < 100)){
            // Likely green purple purple
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
        }
        else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
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

    public Action setIntakeRPM(int rpm) {
        return new InstantAction(() -> {intakeTarget=rpm;});
    }

    public Action setShooterRPM(int rpm) { return new InstantAction(() -> shooterTarget =rpm); }

    public Action shootAction(int shooter) {return new InstantAction(() -> shoot(shooter));}
    public Action loadAction() {return new InstantAction(this::load);}


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