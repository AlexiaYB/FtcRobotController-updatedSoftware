package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutonomousRFspikeTP", group = "Autonomous")
@Disabled
public class AutonomousRFspikeTP extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    Servo armServoBase;
    Servo armServoTop;
    Servo clawServo;
    Servo pixelDropper;
    DcMotor leftSlide;

    // imu
    private IMU imu = null;

    // encoder constants
    double CPR = ((((1+(46/11))) * (1+(46/11))) * 28);
    double CM_DIAMETER = 14;
    double CM_CIRCUMFERENCE = Math.PI * CM_DIAMETER;
    double COUNTS_PER_CM = CPR/CM_CIRCUMFERENCE;
    double CPR_slide = ((((1+(46/11))) * (1+(46/11))) * 28);
    double CMPR_slide= 12;
    double COUNTS_PER_CM_slide = CPR_slide/CMPR_slide;

    // standard drive constants
    static final double STANDARD_DRIVE_SPEED = 0.4;
    static final double STANDARD_TURN_SPEED = 0.35;
    static final double ERROR_TURN_GAIN = 0.03;
    static final double ERROR_DRIVE_GAIN = 0.015;

    // heading constants/variables
    double headingError;
    double HEADING_THRESHOLD = 0.5;

    // april tag drive constants
    final double AT_SPEED_GAIN =  0.01;
    final double AT_STRAFE_GAIN =  0.01;
    final double AT_TURN_GAIN =  0.01;
    final double AT_MAX_AUTO_SPEED = 0.2;
    final double AT_MAX_AUTO_STRAFE = 0.2;
    final double AT_MAX_AUTO_TURN = 0.2;

    // tensorflow variables
    private static final String TFOD_MODEL_ASSET = "modelInitialTeamProp.tflite";
    private static final String[] LABELS = {"teamProp"};
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    boolean initial = true;


    @Override
    public void runOpMode() {
        // initialising variables
        boolean targetFound = false;
        int DESIRED_TAG_ID;

        // from hardware map
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armServoBase = hardwareMap.get(Servo.class, "arm_base");
        armServoTop = hardwareMap.get(Servo.class, "arm_top");
        clawServo = hardwareMap.get(Servo.class, "claw");
        pixelDropper = hardwareMap.get(Servo.class, "pixel_dropper");
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

        leftSlide = hardwareMap.get(DcMotor.class, "slide_drive");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // vision portal initialised
        initVisionPortal();

        // imu
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // set up servo positions
        pixelDropper.setPosition(0.5);
        clawServo.setPosition(1.0);
        armServoBase.setPosition((double) 70/ 180);


        // indicating initialisation complete
        telemetry.addData("Status", "Initialized & Connected");
        telemetry.update();

        waitForStart();
        runtime.reset();
        imu.resetYaw();
        while (opModeIsActive()) {
            if (initial) {
                initial = false;
                xMovement(false, 10, 0);
                // detect from starting location
                runtime.reset();
                int TPDetection = 0;
                while (runtime.seconds() < 3) {
                    TPDetection = recognitionsTfod();
                    if (TPDetection != 0) {
                        break;
                    }
                    sleep(10);
                }
                // on right mark
                if (TPDetection == 0) {
                    DESIRED_TAG_ID = 3;
                    // drive to mark
                    xMovement(false, 4, 0);
                    yMovement(true, 70, 0);
                    turnToHeading(STANDARD_TURN_SPEED, 90);
                    yMovement(true, 34, 90);
                    // place pixel
                    pixelDropper.setPosition(0.0);
                    sleep(1000);
                    // drive back to start
                    yMovement(false, 42, 90);
                    turnToHeading(STANDARD_TURN_SPEED, 0);
                    yMovement(false, 63, 0);
                }
                // on centre mark
                else if (TPDetection == 2) {
                    DESIRED_TAG_ID = 2;
                    // drive to mark
                    xMovement(true, 10, 0);
                    yMovement(true, 84, 0);
                    // place pixel
                    pixelDropper.setPosition(0.0);
                    sleep(1000);
                    // drive back to start
                    yMovement(false, 78, 0);
                    xMovement(false, 10, 0);
                }
                // on left mark
                else {
                    DESIRED_TAG_ID = 1;
                    // drive to mark
                    xMovement(false, 8, 0);
                    yMovement(true, 58, 0);
                    // place pixel
                    pixelDropper.setPosition(0.0);
                    sleep(1000);
                    // drive back to start
                    xMovement(true, 8, 0);
                    yMovement(false, 44, 0);
                }
                // drive to board
                turnToHeading(STANDARD_TURN_SPEED, -90);
                yMovement(true, 46, -90);
                correctHeading(STANDARD_TURN_SPEED, -90);
                armServoBase.setPosition((double) 170 / 180);
                slideMovement(false, 52);
                armServoBase.setPosition((double) 80 / 180);
                armServoTop.setPosition((double) 180 / 180);


                if (DESIRED_TAG_ID == 1) {
                    xMovement(false, 55, 90);
                } else if (DESIRED_TAG_ID == 2) {
                    xMovement(false, 40, 90);
                } else {
                    xMovement(false, 32, 90);

                }
//             april tags
                setManualExposure(6, 250);
                int tagFoundAndLost = 0;
                while (tagFoundAndLost != 2) {
                    moveRobot(0, 0, 0);
                    brakeAll();
                    targetFound = false;
                    desiredTag = null;
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID)) {
                            desiredTag = detection;
                            targetFound = true;
                            tagFoundAndLost = 1;
                            break;
                        }
                    }
                    if (targetFound) {
                        // April tag errors
                        double rangeError = (desiredTag.ftcPose.range - 18.0);
                        double headingError = -desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        double drive = Range.clip(rangeError * AT_SPEED_GAIN, -AT_MAX_AUTO_SPEED, AT_MAX_AUTO_SPEED);
                        double turn = Range.clip(headingError * AT_TURN_GAIN, -AT_MAX_AUTO_TURN, AT_MAX_AUTO_TURN);
                        double strafe = Range.clip(yawError * AT_STRAFE_GAIN, -AT_MAX_AUTO_STRAFE, AT_MAX_AUTO_STRAFE);

                        moveRobot(drive, strafe, turn);
                        if (rangeError < 1) {
                            tagFoundAndLost = 2;
                        }
                    }
                    sleep(10);
                }
                xMovement(false ,10,-90);
                clawServo.setPosition(0.0);
                sleep(5000);
            }
        }

    }// end autonomous mode
    public void yMovement(boolean forwards, double cmDistance, double heading) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * COUNTS_PER_CM);
            double power = STANDARD_DRIVE_SPEED;
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
                double turnSpeed = getSteeringCorrection(heading, ERROR_DRIVE_GAIN);
                // works on speed not encoder position
                if (cmDistance < 0)
                    turnSpeed *= -1.0;
                moveRobot(power,0, turnSpeed);
                telemetry.addData(">", "Robot Heading = %f", getHeading());
                telemetry.update();

            }

            // Stop all motion;
            brakeAll();

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // correct heading
            correctHeading(STANDARD_TURN_SPEED,heading);
        }
    }
    public void xMovement(boolean right, double cmDistance, double heading) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * COUNTS_PER_CM);
            double power = STANDARD_DRIVE_SPEED;
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
                double turnSpeed = getSteeringCorrection(heading, ERROR_DRIVE_GAIN);
                // works on speed not encoder position
                if (cmDistance < 0)
                    turnSpeed *= -1.0;
                moveRobot(power,0, turnSpeed);
                telemetry.addData(">", "Robot Heading = %f", getHeading());
                telemetry.update();

            }

            // Stop all motion;
            brakeAll();

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            correctHeading(STANDARD_TURN_SPEED,heading);

        }
    }
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            double turnSpeed = getSteeringCorrection(heading, ERROR_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0,0, turnSpeed);
            telemetry.addData(">", "Robot Heading = %f", getHeading());
            telemetry.update();
        }

        // Stop all motion;
        moveRobot(0,0,0);
    }
    public void turnToHeading(double maxTurnSpeed, double newHeading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(newHeading, ERROR_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            double turnSpeed = getSteeringCorrection(newHeading, ERROR_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, 0,turnSpeed);
        }

        // Stop all motion;
        moveRobot(0,0,0);
    }
    public void correctHeading(double maxTurnSpeed, double desiredHeading){
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            double turnSpeed = getSteeringCorrection(desiredHeading, ERROR_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0,0, turnSpeed);
        }
        // Stop all motion;
        moveRobot(0,0,0);
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
    public void brakeAll(){
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    public void slideMovement(boolean down, int cmDistance) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * COUNTS_PER_CM_slide);
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
    // vision portal
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        tfod.setMinResultConfidence(0.75f);
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(2);
    }
    private void initVisionPortal(){
        initTfod();
        initAprilTag();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(tfod)
                .addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    private int recognitionsTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() == 0){
            return 0;
        }
        else{
            Recognition recognition = currentRecognitions.get(0);
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            if (x <= 300){
                // to the left mark (centre)
                return 1;

            }
            else{
                // to the right mark (right)
                return 2;
            }
        }

    }
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }


    }

}

