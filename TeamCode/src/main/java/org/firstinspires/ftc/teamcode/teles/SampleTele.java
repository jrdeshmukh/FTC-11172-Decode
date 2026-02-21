//package org.firstinspires.ftc.teamcode.teles;
//
//import static org.firstinspires.ftc.teamcode.config.GreenFN.currentMotif;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ReadWriteFile;
//
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.teamcode.autos.MovementAuto;
//import org.firstinspires.ftc.teamcode.config.GreenFN;
//
//import java.io.File;
//
//@TeleOp
//public class SampleTele extends OpMode {
//    GreenFN g;
//    @Override
//    public void init() {
//
//        g = new GreenFN(hardwareMap);
//        //g.parseMotif();
//    }
//
//    @Override
//    public void loop() {
//        g.update();
//        telemetry.addData("Current motif:", currentMotif);
//        telemetry.update();
//    }
//
//
//
//}
