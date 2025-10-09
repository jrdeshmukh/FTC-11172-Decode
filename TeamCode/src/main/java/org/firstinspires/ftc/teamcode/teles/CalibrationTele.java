package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.BBG;
import org.firstinspires.ftc.teamcode.config.GreenFN;

@Config
@TeleOp
public class CalibrationTele extends OpMode {

    GreenFN g;
    BBG gp1, gp2;

    public static double stopperPos;
    public static double increment = 0.01;
    public static double rpmIncrement = 50;
    @Override
    public void init() {
        g = new GreenFN(hardwareMap);
        g.follower.startTeleopDrive();
        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        stopperPos = g.stopper1.getPosition();
    }

    @Override
    public void loop() {
//        g.update();
//        g.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.27, -gamepad1.right_stick_x * 0.75, true);

        if(gp2.left_bumper()) {
            g.stopper1.setPosition(stopperPos+increment);
            g.stopper2.setPosition(stopperPos+increment);
            g.stopper3.setPosition(stopperPos+increment);

        }
        if(gp2.right_bumper()) {
            g.stopper1.setPosition(stopperPos-increment);
            g.stopper2.setPosition(stopperPos-increment);
            g.stopper3.setPosition(stopperPos-increment);
        }

        g.intake.setPower(gamepad2.left_stick_y);
        g.shooter1.setPower(gamepad2.right_stick_y);
        g.shooter2.setPower(gamepad2.right_stick_y);



        stopperPos = g.stopper1.getPosition();

        telemetry.addData("Current Motif: ", GreenFN.currentMotif);
        telemetry.addData("Shooter RPM: ", g.shooter1.getVelocity());
        telemetry.addData("Shooter Target RPM: ", g.shooterTarget);
        telemetry.addData("Intake RPM: ", g.intake.getVelocity());
        telemetry.addData("Intake Target RPM: ", g.intakeTarget);
        telemetry.addData("Stopper Position: ", g.stopper1.getPosition());
        telemetry.update();
    }
}
