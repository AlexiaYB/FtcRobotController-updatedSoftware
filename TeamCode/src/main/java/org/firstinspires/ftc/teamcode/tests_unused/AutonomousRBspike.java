package org.firstinspires.ftc.teamcode.tests_unused;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "AutonomousRBspike", group = "Autonomous")
@Disabled
public class AutonomousRBspike extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // imu
    private IMU imu = null;

    // encoder constants
    double CPR = ((((1+(46/11))) * (1+(46/11))) * 28);
    double CM_DIAMETER = 14;
    double CM_CIRCUMFERENCE = Math.PI * CM_DIAMETER;
    double COUNTS_PER_CM = CPR/CM_CIRCUMFERENCE;

    // drive constants
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.3;
    static final double     P_TURN_GAIN            = 0.02;
    static final double     P_DRIVE_GAIN           = 0.03;

    // heading constants/variables
    double headingError;
    double HEADING_THRESHOLD = 0.5;

    // tensorflow variables
    private TfodProcessor tfod;
    private VisionPortal visionPortal;


    @Override
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
        // resetting motor encoders to 0
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // tfod initialised
        initTfod();

        // imu
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // indicating initialisation complete
        telemetry.addData("Status", "Initialized & Connected");
        telemetry.update();

        waitForStart();
        runtime.reset();
        imu.resetYaw();
        if (opModeIsActive()) {
            xMovement(false, 23, 0);
            yMovement(true, 30, 0);

            runtime.reset();
            boolean pixelDetection = false;
            while (runtime.seconds() < 2) {
                pixelDetection = telemetryTfod();
                telemetry.addData("pixel detected: ", pixelDetection);
                telemetry.update();
                if (pixelDetection == true) {
                    break;
                }
                sleep(10);
            }
            if (pixelDetection == true) {
                visionPortal.close();
                yMovement(false, 30, 3);
                xMovement(false, 90, 3);
            }
            else {
                xMovement(true, 33, 5);
                yMovement(true, 12, 2);

                runtime.reset();
                while (runtime.seconds() < 2) {
                    pixelDetection = telemetryTfod();
                    telemetry.addData("pixel detected: ", pixelDetection);
                    telemetry.update();
                    if (pixelDetection == true) {
                        break;
                    }
                    sleep(10);
                }
            if (pixelDetection == true){
                visionPortal.close();
                yMovement(false, 40, 0);
                xMovement(false, 120, 0);
            }

            }
        }


    }// end autonomous mode
    public void yMovement(boolean forwards, double cmDistance, double heading) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * COUNTS_PER_CM);
            double power = DRIVE_SPEED;
            if (forwards == false) {
                counts = -counts;
            }

            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + counts);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + counts);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + counts);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + counts);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            moveRobot(power,0,0);

            while (opModeIsActive() && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                    && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                double turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                // works on speed not encoder position
                if (cmDistance < 0)
                    turnSpeed *= -1.0;
                moveRobot(power,0, turnSpeed);
                telemetry.addData(">", "Robot Heading = %f", getHeading());
                telemetry.update();

            }

            // Stop all motion;
            moveRobot(0,0,0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void xMovement(boolean right, double cmDistance, double heading) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * COUNTS_PER_CM);
            double power = DRIVE_SPEED;
            if (right == false) {
                counts = -counts;
            }

            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + counts);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - counts);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - counts);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + counts);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            moveRobot(0,power,0);

            while (opModeIsActive() && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                    && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                double turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                // works on speed not encoder position
                if (cmDistance < 0)
                    turnSpeed *= -1.0;
                moveRobot(power,0, turnSpeed);
                telemetry.addData(">", "Robot Heading = %f", getHeading());
                telemetry.update();

            }

            // Stop all motion;
            moveRobot(0,0,0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            double turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0,0, turnSpeed);
            telemetry.addData(">", "Robot Heading = %f", getHeading());
            telemetry.update();
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }
    public void turnToHeading(double maxTurnSpeed, double newHeading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(newHeading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            double turnSpeed = getSteeringCorrection(newHeading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, 0,turnSpeed);
        }

        // Stop all motion;
        moveRobot(0, 0,0);
    }
    public void correctHeading(double maxTurnSpeed, double desiredHeading){
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            double turnSpeed = getSteeringCorrection(desiredHeading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0,0, turnSpeed);
        }
        // Stop all motion;
        moveRobot(0, 0, 0);
    }
    public void moveRobot(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;


        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        headingError = desiredHeading - getHeading();
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return Range.clip(-1 * headingError * proportionalGain, -1, 1);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void initTfod() {
        tfod = TfodProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

    }
    private boolean telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() == 0){
            return false;
        }
        else{
            return true;
        }
    }
}

