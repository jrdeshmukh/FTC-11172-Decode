package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GreenFN {
    public static double shooterCloseRPM = 4000;
    public static double closeHoodAngle = 80;


    PIDController fly1Controller, fly2Controller, intakeController;
    public static double intakekP, intakekI, intakekD, flykP, flykI, flykD;
    public DcMotorEx flywheel1, flywheel2, intake;

    public static double flywheelTarget, intakeTarget;

    public enum Motifs {
        GPP,
        PGP,
        PPG
    }

    public static Motifs currentApriltag = null;


    public GreenFN(HardwareMap map) {
        flywheel1 = map.get(DcMotorEx.class, "flywheel1");
        flywheel2 = map.get(DcMotorEx.class, "flywheel2");
        intake = map.get(DcMotorEx.class, "intake");
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1Controller = new PIDController(flykP, flykI, flykD);
        fly2Controller = new PIDController(flykP, flykI, flykD);
        intakeController = new PIDController(intakekP, intakekI, intakekD);
    }

    public void update() {
        updatePIDs();
        updateLights();

    }

    public void updatePIDs() {
        double fly1power = fly1Controller.calculate(flywheel1.getVelocity(AngleUnit.DEGREES), flywheelTarget);
        double fly2power = fly2Controller.calculate(flywheel2.getVelocity(AngleUnit.DEGREES), flywheelTarget);

        double intakepower = intakeController.calculate(intake.getVelocity(AngleUnit.DEGREES), intakeTarget);

        flywheel1.setPower(fly1power);
        flywheel2.setPower(fly2power);
        intake.setPower(intakepower);
    }

    //update lights based on color sensor
    public void updateLights() {

    }


    //set currentApriltag based on detected apriltag's id, also stop detecting when you see a motif
    public void updateAprilTagDetection() {

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