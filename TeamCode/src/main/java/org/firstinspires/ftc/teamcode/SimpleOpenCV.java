package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.opencv.core.Rect;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Config
public class SimpleOpenCV extends OpMode {
    private overallEOCVprocessor overallEOCVprocessor;
    private VisionPortal visionPortal;
    public static Rect left = new Rect(0, 55, 120, 170);
    public static Rect right = new Rect(310, 100, 185, 200);
    @Override
    public void init(){
        overallEOCVprocessor = new overallEOCVprocessor(left, right, false);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), overallEOCVprocessor);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    @Override
    public void init_loop(){
        telemetry.addData("Identified", overallEOCVprocessor.getSelection());
        telemetry.addData("Left", overallEOCVprocessor.distanceRectLeft);
        telemetry.addData("Right", overallEOCVprocessor.distanceRectRight);
    }

    @Override
    public void start(){
        visionPortal.stopStreaming();
    }
    @Override
    public void loop(){
        telemetry.addData("Identified", overallEOCVprocessor.getSelection());
    }

}
