package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.config.GreenFN.currentMotif;


import android.graphics.Color;
import android.graphics.Rect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.config.GreenFN;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

import java.io.File;
import java.util.List;

@Autonomous
public class SampleAuto extends OpMode {

    GreenFN g;
    @Override
    public void init() {

        g = new GreenFN(hardwareMap);
        currentMotif = null;

    }
    @Override
    public void loop() {
        g.update();
        List<ColorBlobLocatorProcessor.Blob> purple = g.colorLocator.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> green = g.colorLocator2.getBlobs();
        int blobCount = purple.size() + green.size();
        Size myBoxFitSize;
        for(ColorBlobLocatorProcessor.Blob b : purple)
        {
            RotatedRect boxFit = b.getBoxFit();
            myBoxFitSize = boxFit.size;
            telemetry.addData("width", myBoxFitSize.width);
            telemetry.addData("height", myBoxFitSize.height);
            telemetry.addData("angle", boxFit.angle);
            if(myBoxFitSize.width/ myBoxFitSize.height > 3.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.35 ){
                blobCount = blobCount + 1;
            }
            else if (myBoxFitSize.width/ myBoxFitSize.height > 5.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.18) {
                blobCount = blobCount + 2;
            }
            else if (myBoxFitSize.width/ myBoxFitSize.height > 7.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.135) {
                blobCount = blobCount + 3;
            }
            else if (myBoxFitSize.width/ myBoxFitSize.height > 9.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.11) {
                blobCount = blobCount + 4;
            }
            else if (myBoxFitSize.width/ myBoxFitSize.height > 11.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.09) {
                blobCount = blobCount + 5;
            }
            else if (myBoxFitSize.width/ myBoxFitSize.height > 13.8 || myBoxFitSize.width/ myBoxFitSize.height < 0.078) {
                blobCount = blobCount + 6;
            }
        }
        Size myBoxFitSize2;
        for(ColorBlobLocatorProcessor.Blob b : green)
        {
            RotatedRect boxFit = b.getBoxFit();
            myBoxFitSize2 = boxFit.size;
            telemetry.addData("width2", myBoxFitSize2.width);
            telemetry.addData("height2", myBoxFitSize2.height);
            telemetry.addData("angle2", boxFit.angle);

            if(myBoxFitSize2.width/ myBoxFitSize2.height > 3.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.35 ){
                blobCount = blobCount+1;
            }
            else if (myBoxFitSize2.width/ myBoxFitSize2.height > 5.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.18) {
                blobCount = blobCount+2;
            }
            else if (myBoxFitSize2.width/ myBoxFitSize2.height > 7.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.135) {
                blobCount = blobCount+3;
            }
            else if (myBoxFitSize2.width/ myBoxFitSize2.height > 9.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.11) {
                blobCount = blobCount + 4;
            }
            else if (myBoxFitSize2.width/ myBoxFitSize2.height > 11.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.09) {
                blobCount = blobCount + 5;
            }
            else if (myBoxFitSize2.width/ myBoxFitSize2.height > 13.8 || myBoxFitSize2.width/ myBoxFitSize2.height < 0.078) {
                blobCount = blobCount + 6;
            }
        }

        telemetry.addData("blobs detected", blobCount);
        telemetry.addData("Current motif:", currentMotif);
        telemetry.update();
        g.update();
    }
}
