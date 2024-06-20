package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class servoTester extends LinearOpMode {
    public static double clawPos = 0.0;
    public static double armTopPos = 0.0;
    public static double armBasePos = 0.0;
    @Override
    public void runOpMode() {
        Servo claw = hardwareMap.servo.get("claw");
        Servo armTop = hardwareMap.servo.get("armTop");
        Servo armBase = hardwareMap.servo.get("armBase");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            claw.setPosition(clawPos);
            armBase.setPosition(armBasePos);
            armTop.setPosition(armTopPos);
        }
    }
}
