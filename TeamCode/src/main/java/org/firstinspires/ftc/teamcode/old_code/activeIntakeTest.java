package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "activeIntakeTest", group = "Concept")
@Disabled
public class activeIntakeTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // Define class members
    CRServo ATServo;



    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        ATServo = hardwareMap.get(CRServo.class, "at_servo");

        telemetry.addData(">", "press start" );
        telemetry.update();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            ATServo.setPower(1.0);
        }
    }
}


