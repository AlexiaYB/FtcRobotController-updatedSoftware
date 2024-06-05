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

@TeleOp
public class DriverCentric extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Gamepad current = new Gamepad();
    Gamepad previous = new Gamepad();

    @Override
    public void runOpMode(){
        // initialise
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LeftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LeftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RightBack");
        DcMotor rightHanging = hardwareMap.dcMotor.get("RightHanging");
        DcMotor leftHanging = hardwareMap.dcMotor.get("LeftHanging");
        Servo drone = hardwareMap.servo.get("drone");
        Servo claw = hardwareMap.servo.get("claw");
        Servo armTop = hardwareMap.servo.get("armTop");
        Servo armBase = hardwareMap.servo.get("armBase");
        CRServo activeIntake = hardwareMap.crservo.get("activeIntake");

        // set up servo positions
        drone.setPosition(0.47);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // get gamepad values
            previous.copy(current);
            current.copy(gamepad2);
            double y = -current.left_stick_y;
            double x = current.left_stick_x;
            double rx = current.right_stick_x;
//            // drive testing
//            if(gamepad2.options){
//               imu.resetYaw();
//            }

//           // drone firing
//            if(gamepad2.triangle){
//                drone.setPosition(0);
//            }

            // hanging motors
//            if(current.square){
//                rightHanging.setPower(1);
//                leftHanging.setPower(1);
//            } else if (current.square != true && previous.square == true) {
//                rightHanging.setPower(0);
//                leftHanging.setPower(0);
//            }

//            // drive train movement
//            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

            // intake
            // 1
            if (current.cross && !previous.cross){
                armBase.setPosition(1);
                claw.setPosition(0.12);
                armTop.setPosition(0.32);

            }
            // 2
            if (current.circle && !previous.circle){
                armBase.setPosition(0.90);
                armTop.setPosition(0.45);
            }
            // active intake
            if (current.dpad_right) {
                activeIntake.setPower(-1.0);
            }
            else if (current.dpad_left) {
                activeIntake.setPower(1.0);
            }
            else{
                activeIntake.setPower(0.0);
            }
//            // close claw & move up
            if (current.square && !previous.square){
                claw.setPosition(1.0);
                sleep(600);
                armTop.setPosition(0.70);
                runtime.reset();
                while (runtime.seconds() < 2) {
                    activeIntake.setPower(-0.4);
                }
            }

        }
    }
}


