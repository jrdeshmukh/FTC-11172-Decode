package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.util.List;


@Config
public class GreenFN {
    public static double shooterCloseRPM = 4000;
    public static double closeHoodAngle = 80;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    PIDController fly1Controller, fly2Controller, intakeController;
    public static double intakekP = 0.00017, intakekI, intakekD, flykP, flykI, flykD, intakeKF = 0.000285, flyKF = 0.000285;
    public DcMotorEx flywheel1, flywheel2, intake;
    public ColorSensor colorSensor1, colorSensor2, colorSensor3;
    RevBlinkinLedDriver lights;
    public static double flywheelTarget, intakeTarget;
    //-1 is place holder
    public static int automotif = -1;

    public double intakePower, flywheel1Power, flywheel2Power;

    public enum Motifs {
        GPP,
        PGP,
        PPG
    }

    public static Motifs currentMotif = null;


    public GreenFN(HardwareMap map) {
        flywheel1 = map.get(DcMotorEx.class, "flywheel1");
        flywheel2 = map.get(DcMotorEx.class, "flywheel2");
        intake = map.get(DcMotorEx.class, "intake");
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1Controller = new PIDController(flykP, flykI, flykD);
        fly2Controller = new PIDController(flykP, flykI, flykD);
        intakeController = new PIDController(intakekP, intakekI, intakekD);
//        colorSensor1 = map.get(ColorSensor.class, "colorSensor1");
//        colorSensor2 = map.get(ColorSensor.class, "colorSensor2");
//        colorSensor3 = map.get(ColorSensor.class, "colorSensor3");
//        lights = map.get(RevBlinkinLedDriver.class, "lights");
//        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                map.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    /**
     * Add telemetry about AprilTag detections.
     */

    public void update() {
        updatePIDs();
        //updateLights();
        //updatePIDs();
        //updateLights();
        if(currentMotif == null){
//            updateAprilTagDetection();
        }
//        else{
//            File file = AppUtil.getInstance().getSettingsFile("motif.txt");
//            switch(currentMotif) {
//                case GPP:
//                    ReadWriteFile.writeFile(file, "GPP");
//                    break;
//                case PGP:
//                    ReadWriteFile.writeFile(file, "PGP");
//                    break;
//                case PPG:
//                    ReadWriteFile.writeFile(file, "PPG");
//                    break;
//            }
//            visionPortal.close();
//        }
    }

//    public void parseMotif() {
//        File file = AppUtil.getInstance().getSettingsFile("motif.txt");
//        String input = ReadWriteFile.readFile(file).trim();
//        ReadWriteFile.writeFile(file, "null");
//        switch (input) {
//            case "GPP":
//                currentMotif = Motifs.GPP;
//                break;
//            case "PGP":
//                currentMotif = Motifs.PGP;
//                break;
//            case "PPG":
//                currentMotif = Motifs.PPG;
//                break;
//            case "null":
//                currentMotif = null;
//                break;
//            case "nul":
//                currentMotif = null;
//                break;
//        }
//    }

    public void updatePIDs() {
        flywheel1Power = fly1Controller.calculate(flywheel1.getVelocity(AngleUnit.DEGREES), flywheelTarget) + flywheel1.getVelocity()*flyKF;
        flywheel2Power = fly2Controller.calculate(flywheel2.getVelocity(AngleUnit.DEGREES), flywheelTarget) + flywheel2.getVelocity()*flyKF;

        intakePower = intakeController.calculate(intake.getVelocity(AngleUnit.DEGREES), intakeTarget) + intake.getVelocity()*intakeKF;

        flywheel1.setPower(flywheel1Power);
        flywheel2.setPower(flywheel2Power);
        intake.setPower(intakePower);

        intakeController.setPID(intakekP, intakekI, intakekD);
    }

    //update lights based on color sensor
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


    //set currentApriltag based on detected apriltag's id, also stop detecting when you see a motif
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
  // end for() loop
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