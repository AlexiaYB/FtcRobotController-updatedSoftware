package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp1", group="TeleOp1Group")
@Disabled
public class TeleOp1  extends LinearOpMode {
    // declaring variables
    private ElapsedTime runtime = new ElapsedTime();
    // drive train motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

//    Servo armServoBase;
//    Servo armServoTop;
//    Servo clawServo;
    CRServo Aintake;
//    DcMotor leftSlide;
    DcMotor rightHanging;
    DcMotor leftHanging;
    Servo drone;
    Servo hangingLeft;
    Servo hangingRight;

    Gamepad current = new Gamepad();
    Gamepad previous = new Gamepad();

    boolean gripper_open = false;
    boolean hanging = false;


    int preset = 0;
    boolean presetChange = false;

//    double CPR = ((((1+(46/11))) * (1+(46/11))) * 28);
//    double CMPR= 12;
//    double COUNTS_PER_CM = CPR/CMPR;

    public void runOpMode() {
        // initialising variables
        // from hardware map
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightHanging = hardwareMap.get(DcMotor.class, "right_hanging_drive");
        leftHanging = hardwareMap.get(DcMotor.class, "left_hanging_drive");


        // accounting for reversed motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

//        armServoBase = hardwareMap.get(Servo.class, "arm_base");
//        armServoTop = hardwareMap.get(Servo.class, "arm_top");
//        clawServo = hardwareMap.get(Servo.class, "claw");
//        leftSlide = hardwareMap.get(DcMotor.class, "slide_drive");
        Aintake = hardwareMap.get(CRServo.class, "intake");
        drone = hardwareMap.get(Servo.class, "drone_shooter");
        hangingLeft = hardwareMap.get(Servo.class, "right_hanging_servo");
        hangingRight = hardwareMap.get(Servo.class, "left_hanging_servo");


//        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            // get gripper to right angle
//            if(current.square && !previous.square){
//                armServoBase.setPosition((double) 80/ 180);
//                armServoTop.setPosition((double) 180 / 180);
//            }
            if (current.square && !previous.square) {
                drone.setPosition(0.0);
            }

            // open/close gripper
//            if (current.cross && !previous.cross) {
//                if (gripper_open) {
//                    clawServo.setPosition(1.0);
//                    gripper_open = false;
//                } else {
//                    clawServo.setPosition(0.0);
//                    gripper_open = true;
//
//                }
//            }
            // move slide up board (hold position?)
//            if (current.dpad_up) {
//                leftSlide.setPower(0.5);
//            } else if (current.dpad_down) {
//                leftSlide.setPower(-0.5);
//            } else {
//                leftSlide.setPower(0);
//            }
            // bring slide back to active intake!!!!
//            if(current.triangle && )


//            // active intake
            if (current.cross) {
                Aintake.setPower(1.0);
            } else if (current.triangle){
                Aintake.setPower(0.0);
            }
            else {
                Aintake.setPower(-1.0);
            }

            // shoot drone
//            if (current.right_bumper){
//                if (current.square) {
//                    drone.setPosition(1.0);
//                }
            if (current.circle) {
                hangingRight.setPosition(0.0);
                hangingLeft.setPosition(1.0);
                hanging = true;
            }


            // Drive Train:
            // calculating power to wheels
            double y = -current.left_stick_y; // y-stick reversed
            double x = current.left_stick_x * 1.1; // Counteracts imperfect strafing
            double rx = current.right_stick_x;


            // braking
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // Send calculated power to wheels
            // ensures that power doesn't exceed abs(1)
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            // slow down for careful movement
            if (current.left_bumper) {
                leftFrontPower = leftFrontPower * 0.6;
                leftBackPower = leftBackPower * 0.6;
                rightFrontPower = rightFrontPower * 0.6;
                rightBackPower = rightBackPower * 0.6;
            }
            if (hanging) {
                rightHanging.setPower(-current.right_stick_y);
                leftHanging.setPower(-current.right_stick_y);
            }

            // sends power to the wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

        }
    }



//    public void slideMovement(boolean down, int cmDistance) {
//        if (opModeIsActive()) {
//            int counts = (int) (cmDistance * COUNTS_PER_CM);
//            if (down){
//                counts = -counts;
//            }
//            double power = 0.4;
//            leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + counts);
//            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftSlide.setPower(power);
//
//            while (opModeIsActive() && (leftSlide.isBusy())) {}
//            leftSlide.setPower(0);
//            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//    public void slideHoldPosition() {
//        if (opModeIsActive()) {
//            double power = 0.2;
//            leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
//            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftSlide.setPower(power);
//            sleep(50);
//            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
}
