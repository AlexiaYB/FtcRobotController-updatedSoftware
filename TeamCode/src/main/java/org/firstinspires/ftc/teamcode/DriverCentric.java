package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class DriverCentric extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime overallTimer = new ElapsedTime();
    boolean slideDown = false;

    Gamepad current1 = new Gamepad();
    Gamepad previous1 = new Gamepad();
    Gamepad current2 = new Gamepad();
    Gamepad previous2 = new Gamepad();
    int driveMultiplierPlace = 0;
    List<Double> multipliers = Arrays.asList(1.0,0.75,0.5,0.25);


    @Override
    public void runOpMode(){
        // initialise
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LeftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LeftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RightBack");
        DcMotor rightHanging = hardwareMap.dcMotor.get("RightHanging");
        DcMotor leftHanging = hardwareMap.dcMotor.get("LeftHanging");
        DcMotor ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        Servo drone = hardwareMap.servo.get("drone");
        Servo claw = hardwareMap.servo.get("claw");
        Servo armTop = hardwareMap.servo.get("armTop");
        Servo armBase = hardwareMap.servo.get("armBase");
        Servo rightFlip = hardwareMap.servo.get("rightFlip");
        Servo leftFlip = hardwareMap.servo.get("leftFlip");
        CRServo activeIntake = hardwareMap.crservo.get("activeIntake");

        // set up servo positions
        drone.setPosition(0.47);
        rightFlip.setPosition(0.3);
        leftFlip.setPosition(0.35);

        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();
        overallTimer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // get gamepad values
            previous2.copy(current2);
            current2.copy(gamepad2);
            previous1.copy(current1);
            current1.copy(gamepad1);
            double y = -current2.left_stick_y;
            double x = current2.left_stick_x;
            double rx = current2.right_stick_x;
            // value adjusts
            // in case imu shifts slightly
            if(current2.options){
                imu.resetYaw();
            }
            // endgame things
            if(overallTimer.seconds() > 90){
                // drone firing & releasing hooks
                if(current2.triangle){
                    drone.setPosition(0);
                    rightFlip.setPosition(1.0);
                    leftFlip.setPosition(1.0);
                }

                // hanging motors
                if (current2.left_bumper) {
                    leftHanging.setPower(1);
                }
                if (current2.right_bumper) {
                    rightHanging.setPower(1);
                }
                if (current2.left_trigger == 1) {
                    leftHanging.setPower(-1);
                }
                if (current2.right_trigger == 1) {
                    rightHanging.setPower(-1);
                } else {
                    rightHanging.setPower(0);
                    leftHanging.setPower(0);
                }
            }
            // active intake operation
            if (current2.dpad_right) {
                activeIntake.setPower(-1.0);
            }
            else if (current2.dpad_left) {
                activeIntake.setPower(1.0);
            }
            else{
                activeIntake.setPower(0.0);
            }
            // driver multipliers
            if(current2.left_stick_button && !previous2.left_stick_button && driveMultiplierPlace != 0){
                driveMultiplierPlace --;
            }
            if(current2.right_stick_button && !previous2.right_stick_button && driveMultiplierPlace != 3){
                driveMultiplierPlace ++;
            }

//            // drive train movement
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * multipliers.get(driveMultiplierPlace));
            backLeftMotor.setPower(backLeftPower * multipliers.get(driveMultiplierPlace));
            frontRightMotor.setPower(frontRightPower* multipliers.get(driveMultiplierPlace));
            backRightMotor.setPower(backRightPower * multipliers.get(driveMultiplierPlace));

            // Arm & outtake gamepad1 controlled
            // move slide
            if(slideDown == false) {
                // move slide up board & hold position
                if (current2.dpad_up && ArmMotor.getCurrentPosition() < 5800) {
                    ArmMotor.setPower(0.5);
                } else if (current2.dpad_down) {
                    ArmMotor.setPower(-0.5);
                } else {
                    ArmMotor.setPower(0.001);
                }
                // continue movement in
            } else if (ArmMotor.getCurrentPosition() > 150){
                ArmMotor.setPower(-1);
                // end movement in
            } else{
                slideDown = false;
                ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armBase.setPosition(0.90);
                armTop.setPosition(0.52);
                ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // intake to outtake
            // start movement in
            if (current2.circle && !previous2.circle){
                armBase.setPosition(1);
                claw.setPosition(0.12);
                armTop.setPosition(0.45);
                slideDown = true;
            }
            // close claw for movement out
            if (current2.square && !previous2.square){
                armBase.setPosition(0.85);
                claw.setPosition(1.0);
                armBase.setPosition(1.0);
            }
            // set claw to position on board
            if(ArmMotor.getCurrentPosition() > 2650 && slideDown == false){
                armBase.setPosition(0.70);
                armTop.setPosition(0.48);
            }
            // open claw
            if(current2.cross && !previous2.cross){
                claw.setPosition(0.05);
            }
            telemetry.addData("counts: ",ArmMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}


