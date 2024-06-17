package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.opencv.core.Rect;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class SimpleOpenCV extends OpMode {
    private overallEOCVprocessor overallEOCVprocessor;
    private VisionPortal visionPortal;
    @Override
    public void init(){
        overallEOCVprocessor = new overallEOCVprocessor(new Rect(70, 0, 100, 83), new Rect(445, 0, 100, 150));
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), overallEOCVprocessor);
    }
    @Override
    public void init_loop(){telemetry.addData("Identified", overallEOCVprocessor.getSelection());}

    @Override
    public void start(){
        visionPortal.stopStreaming();
    }
    @Override
    public void loop(){
        telemetry.addData("Identified", overallEOCVprocessor.getSelection());
    }

}
