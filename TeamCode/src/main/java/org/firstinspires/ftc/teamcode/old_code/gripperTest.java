package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "gripperTest", group = "Concept")
@Disabled
public class gripperTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    double CPR = ((((1+(46/11))) * (1+(46/11))) * 28);
    double CMPR= 12;
    double COUNTS_PER_CM = CPR/CMPR;

    // Define class members
    Servo armServoBase;
    Servo armServoTop;
    Servo clawServo;
    DcMotor leftSlide;

    Gamepad current = new Gamepad();
    Gamepad previous = new Gamepad();

    boolean gripper_open = false;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        armServoBase = hardwareMap.get(Servo.class, "arm_base");
        armServoTop = hardwareMap.get(Servo.class, "arm_top");
        clawServo = hardwareMap.get(Servo.class, "claw");
        leftSlide = hardwareMap.get(DcMotor.class, "slide_drive ");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawServo.setPosition(1.0);

        // Wait for the start button
        telemetry.addData(">", "press start" );
        telemetry.update();
        waitForStart();
        runtime.reset();


        while (opModeIsActive()){
            previous.copy(current);
            current.copy(gamepad2);
            if(current.square) {
//                // run to preset 1
                slideMovement(false,52);
                armServoBase.setPosition((double) 80/ 180);
                armServoTop.setPosition((double) 180 / 80);
            }
            if (current.triangle){
                armServoBase.setPosition((double) 80/ 180);
                armServoTop.setPosition((double) 180 / 80);
                sleep(100);
                slideconstMovment(16);
                armServoBase.setPosition((double) 155/180);
                armServoTop.setPosition((double) 180/180);
                sleep(100);
                slideconstMovment(8);

            }

            if(current.cross && !previous.cross){
                    if (gripper_open) {
                        clawServo.setPosition(1.0);
                        gripper_open = false;
                    } else {
                        clawServo.setPosition(0.0);
                        gripper_open = true;

                }
            }
        }
    }
    public void slideMovement(boolean down, int cmDistance) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * COUNTS_PER_CM);
            if (down){
                counts = -counts;
            }
            double power = 0.4;
            leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + counts);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(power);

            while (opModeIsActive() && (leftSlide.isBusy())) {}
            leftSlide.setPower(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void slideconstMovment(int cmDistance) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * COUNTS_PER_CM);
            double power = 0.4;
            leftSlide.setTargetPosition(counts);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(power);

            while (opModeIsActive() && (leftSlide.isBusy())) {}
            leftSlide.setPower(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}

