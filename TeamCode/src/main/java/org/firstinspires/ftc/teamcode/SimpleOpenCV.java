package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class SimpleOpenCV extends OpMode {
    private DrawRectangleProcessor drawRectangleProcessor;
    private VisionPortal visionPortal;
    @Override
    public void init(){
        drawRectangleProcessor = new DrawRectangleProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), drawRectangleProcessor);
    }
    @Override
    public void init_loop(){telemetry.addData("Identified", drawRectangleProcessor.getSelection());}

    @Override
    public void start(){
        visionPortal.stopStreaming();
    }
    @Override
    public void loop(){
        telemetry.addData("Identified", drawRectangleProcessor.getSelection());
    }

}
