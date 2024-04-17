package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="OutakeTest", group="Tests")
public class OutakeTest extends LinearOpMode {
    // declaring variables
    private ElapsedTime runtime = new ElapsedTime();
    // drive train motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    Servo armServoBase;
    Servo armServoTop;
    Servo clawServo;
    CRServo Aintake;
    DcMotor leftSlide;

    Gamepad current = new Gamepad();
    Gamepad previous = new Gamepad();

    boolean gripper_open = false;

    double CPR = ((((1+(46/11))) * (1+(46/11))) * 28);
    double CMPR= 12;
    double COUNTS_PER_CM = CPR/CMPR;

    public void runOpMode() {
        // initialising variables
        // from hardware map
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        // accounting for reversed motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armServoBase = hardwareMap.get(Servo.class, "arm_base");
        armServoTop = hardwareMap.get(Servo.class, "arm_top");
        clawServo = hardwareMap.get(Servo.class, "claw");
        leftSlide = hardwareMap.get(DcMotor.class, "slide_drive");
        Aintake = hardwareMap.get(CRServo.class, "intake");


        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        clawServo.setPosition(1.0);


        // indicating initialisation complete
        telemetry.addData("Status", "Initialized & Connected");
        telemetry.update();


        // when start pressed
        waitForStart();
        runtime.reset();

        // run opMode
        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad2);
            // open/close gripper
//            if (current.cross && !previous.cross) {
//                if (gripper_open) {
//                    clawServo.setPosition(1.0);
//                    gripper_open = false;
//                } else {
//                    clawServo.setPosition(0.0);
//                    gripper_open = true;
//                }
//            }
            // integral set 1
            if (current.cross && !previous.cross){
                armServoBase.setPosition(0.96);
                clawServo.setPosition(0.12);
                armServoTop.setPosition(0.32);

            }
            // integral set 2
            if (current.circle && !previous.circle){
                armServoBase.setPosition(0.76);
                clawServo.setPosition(0.12);
                armServoTop.setPosition(0.40);
            }
            // close claw
            if (current.square && !previous.square){
                clawServo.setPosition(1.0);
            }
            // active intake
            if (current.dpad_right) {
                Aintake.setPower(1.0);
            }
            else if (current.dpad_left) {
                Aintake.setPower(-1.0);
            }
            else{
                Aintake.setPower(0.0);
            }


            // move slide up board (hold position?)
//            if (current.dpad_up) {
//                leftSlide.setPower(0.5);
//            } else if (current.dpad_down) {
//                leftSlide.setPower(-0.5);
//            } else {
//                leftSlide.setPower(0.001);
//            }


//            // active intake
//            if (current.cross) {
//                Aintake.setPower(1.0);
//            }
//            else if (current.triangle) {
//                Aintake.setPower(-1.0);
//            }
//            else{
//                Aintake.setPower(0.0);
//            }

        }
    }
}

