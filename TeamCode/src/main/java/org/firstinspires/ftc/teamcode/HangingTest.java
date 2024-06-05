package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Hanging Test", group = "Concept")

public class HangingTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // initialising motors so they don't throw an error
        // from hardware map
        DcMotor rightHanging = hardwareMap.get(DcMotor.class, "RightHanging");
        DcMotor leftHanging  = hardwareMap.get(DcMotor.class, "LeftHanging");
        // Wait for the start button
        telemetry.addData("Status", "Initialized & Connected");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            rightHanging.setPower(-gamepad2.right_stick_y);
            leftHanging.setPower(-gamepad2.left_stick_y);
        }
    }
}
