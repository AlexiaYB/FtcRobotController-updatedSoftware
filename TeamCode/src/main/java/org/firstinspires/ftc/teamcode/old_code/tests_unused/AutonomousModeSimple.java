package org.firstinspires.ftc.teamcode.old_code.tests_unused;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "AutonomousModeSimple", group = "Autonomous")
@Disabled
public class AutonomousModeSimple extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // encoder constants
    double CPR = ((((1+(46/11))) * (1+(46/11))) * 28);
    double cmDiameter = 14;
    double cmCircumference = Math.PI * cmDiameter;
    double countsPcm = CPR/cmCircumference;
    double inchDiameter = 5.512;
    double inchCircumference = Math.PI * inchDiameter;
    double countsPinch = CPR/inchCircumference;


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


        // indicating initialisation complete
        telemetry.addData("Status", "Initialized & Connected");
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            yMovement(true, 20);
            sleep(1000);
            yMovement(false,20);
            sleep(1000);
            xMovement(true,20);
            sleep(1000);
            xMovement(false,20);
            sleep(1000);
        }


    }// end autonomous mode

    public void yMovement(boolean forwards, double cmDistance) {
        if (opModeIsActive()) {
            int counts = (int) (cmDistance * countsPcm);
            double power;
            if (forwards == true) {
                power = 0.5;

            } else {
                power = -0.5;
                counts = -counts ;
            }

            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + counts);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + counts);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + counts);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + counts);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

//            leftFrontDrive.setPower(0);
//            rightFrontDrive.setPower(0);
//            leftBackDrive.setPower(0);
//            rightBackDrive.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            sleep(250);
        }
    }
    public void xMovement(boolean left, double cmDistance) {
        int counts = (int) (cmDistance * countsPcm);
        double  power;
        if(left == true){
            power = 0.5;
        }
        else{
            power = -0.5;
            counts = -counts ;

        }

        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() - counts);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + counts);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + counts);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() - counts);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);

    }
}

