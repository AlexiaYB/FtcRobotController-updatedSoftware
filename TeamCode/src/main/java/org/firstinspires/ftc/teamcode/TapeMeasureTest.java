package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    @TeleOp(name = "hangingTest", group = "Concept")

    public static class hangingTest extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();

        // Define class members
        DcMotor hangingMotorLeft;
        DcMotor hangingMotorRight;


        @Override
        public void runOpMode() {

            // Connect to servo (Assume Robot Left Hand)
            // Change the text in quotes to match any servo name on your robot.
            hangingMotorLeft = hardwareMap.get(DcMotor.class, "hanging_right");
            hangingMotorRight = hardwareMap.get(DcMotor.class, "hanging_left");
            hangingMotorRight.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData(">", "press start");
            telemetry.update();
            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                double rightPower = gamepad2.right_stick_y;
                double leftPower = gamepad2.left_stick_y;
                hangingMotorRight.setPower(rightPower);
                hangingMotorLeft.setPower(leftPower);
            }
        }
    }
}
