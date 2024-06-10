package org.firstinspires.ftc.teamcode.old_code.tests_unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="EncoderMoveTest", group="TestGroup")
@Disabled
public class EncoderMoveTest extends LinearOpMode{
    // declaring variables
    private ElapsedTime runtime = new ElapsedTime();
    // drive train motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    // encoder constants
    double cmDiameter = 14;
    double cmCircumference = Math.PI * cmDiameter;
    double inchDiameter = 5.512;
    double inchCircumference = Math.PI * inchDiameter;
    double CPR = ((((1+(46/11))) * (1+(46/11))) * 28);
    double countsPcm = CPR/cmCircumference;

    @Override
    public void runOpMode() {
        // initialising variables

        // from hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // accounting for reversed motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // resetting motor encoders to 0
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // indicating initialisation complete
        telemetry.addData("Status", "Initialized & Connected");
        telemetry.update();



        // when start pressed
        waitForStart();
        runtime.reset();

        // run opMode
        while (opModeIsActive()) {
            // Get the current position of the motor
//            int LFDencoder = leftFrontDrive.getCurrentPosition();
//            int RFDencoder = rightFrontDrive.getCurrentPosition();
//            int LBDencoder = leftBackDrive.getCurrentPosition();
//            int RBDencoder = rightBackDrive.getCurrentPosition();
//            telemetry.addData("start:", "%7d,%7d,%7d,%7d",LFDencoder,RFDencoder,LBDencoder,RBDencoder);
//
//            int counts = (int) (20 * countsPcm);
//            double power  = 0.5;
//
//
//            leftFrontDrive.setTargetPosition(counts);
//            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightFrontDrive.setTargetPosition(counts);
//            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackDrive.setTargetPosition(counts);
//            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightBackDrive.setTargetPosition(counts);
//            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            telemetry.addData("target:", "%7d,%7d,%7d,%7d",leftFrontDrive.getTargetPosition(),rightFrontDrive.getTargetPosition(),leftBackDrive.getTargetPosition(),rightBackDrive.getTargetPosition());
//
//            leftFrontDrive.setPower(power);
//            rightFrontDrive.setPower(power);
//            leftBackDrive.setPower(power);
//            rightBackDrive.setPower(power);
//
//            LFDencoder = leftFrontDrive.getCurrentPosition();
//            RFDencoder = rightFrontDrive.getCurrentPosition();
//            LBDencoder = leftBackDrive.getCurrentPosition();
//            RBDencoder = rightBackDrive.getCurrentPosition();
//            telemetry.addData("end:", "%7d,%7d,%7d,%7d",LFDencoder,RFDencoder,LBDencoder,RBDencoder);
//            telemetry.update();
            yMovement(true,30);

        }
    }
    public void yMovement(boolean forwards, double cmDistance) {
        int counts = (int) (cmDistance * countsPcm);
        double  power;
        if(forwards){
            power = 0.5;
        }
        else{
            power = -0.5;
        }

        leftFrontDrive.setTargetPosition(counts);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setTargetPosition(counts);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setTargetPosition(counts);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setTargetPosition(counts);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }


}
