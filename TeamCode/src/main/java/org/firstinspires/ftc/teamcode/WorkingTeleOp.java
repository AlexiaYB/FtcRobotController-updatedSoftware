package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="WorkingTeleOp", group="TeleOp1Group")
// @Disabled
public class WorkingTeleOp  extends LinearOpMode {
    // declaring variables
    private ElapsedTime runtime = new ElapsedTime();
    // drive train motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    Gamepad current = new Gamepad();
    Gamepad previous = new Gamepad();

    boolean gripper_open = false;
    boolean hanging = false;

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

            // sends power to the wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            telemetry.addData("left bumper", current.left_bumper);
            telemetry.update();
        }

    }
}

