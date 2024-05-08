package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;



public class TapeMeasureTest extends LinearOpMode {
    CRServo ATapeMeasure;
    public void runOpMode() {

        ATapeMeasure = hardwareMap.get(CRServo.class, "TapeMeasure");
        while (opModeIsActive()) {
            if (gamepad2.cross) {
                ATapeMeasure.setPower(1.0);
            } else if (gamepad2.triangle) {
                ATapeMeasure.setPower(0.0);
            } else {
                ATapeMeasure.setPower(-1.0);
            }

        }
    }
}
