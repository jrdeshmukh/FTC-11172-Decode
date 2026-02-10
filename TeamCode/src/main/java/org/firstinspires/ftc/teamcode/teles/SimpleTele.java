package org.firstinspires.ftc.teamcode.teles;

import static org.firstinspires.ftc.teamcode.config.GreenFN.currentMotif;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.autos.MovementAuto;
import org.firstinspires.ftc.teamcode.config.GreenFN;

import java.io.File;

@TeleOp
public class SimpleTele extends OpMode {
    public class MechDrive {
        public DcMotor Fl;
        public DcMotor Fr;
        public DcMotor Bl;
        public DcMotor Br;

        public MechDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
            Fl = fl;
            Fr = fr;
            Bl = bl;
            Br = br;
        }
        public void drive(double forward, double strafe, double turn) {
            if (turn != 0) {
                Fl.setPower(-turn);
                Fr.setPower(turn);
                Fr.setPower(-turn);
                Fr.setPower(turn);
            } else {
                double r = Math.hypot(strafe, forward);
                double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
                double rightX = turn;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;
                Fl.setPower(v1);
                Fr.setPower(v2);
                Fr.setPower(v3);
                Fr.setPower(v4);
            }
        }


    }

    public MechDrive bot;
    public DcMotor Fl;
    public DcMotor Fr;
    public DcMotor Bl;
    public DcMotor Br;
    @Override
    public void init() {
        Fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        Fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        Bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        Br = hardwareMap.get(DcMotorEx.class, "backRight");
        //Reverse motor if needed
        //Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bot = new MechDrive(Fl, Fr, Br, Bl);
    }

    @Override
    public void loop() {
        bot.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
