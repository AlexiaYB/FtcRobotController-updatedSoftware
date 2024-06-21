package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.overallEOCVprocessor.Selected;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;


@Autonomous(name = "autoBF", group = "Autonomous")
@Config
public class autoBF extends LinearOpMode {
    // adjustment constants!!!!!!
    // true if alliance partner has placed a yellow pixel on the board before us
    boolean partnerPlaced = false;
    // 0 means stay at board, 1 means to the pointy corner, 2 means to the square corner
    int parkingLocation = 0;


    private ElapsedTime visionTimer = new ElapsedTime();
    private overallEOCVprocessor overallEOCVprocessor;
    private VisionPortal visionPortal;
    public static Rect left = new Rect(0, 45, 120, 180);
    public static Rect right = new Rect(320, 90, 180, 200);
    double CPR_slide = ((((1+(46/11))) * (1+(46/11))) * 28);
    double CMPR_slide= 12;
    double COUNTS_PER_CM_slide = CPR_slide/CMPR_slide;
    private DcMotor slide = null;
    @Override
    public void runOpMode() {
        // initialise
        slide = hardwareMap.dcMotor.get("ArmMotor");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo pixelDropper = hardwareMap.servo.get("pixelDropper");
        Servo claw = hardwareMap.servo.get("claw");
        Servo armTop = hardwareMap.servo.get("armTop");
        Servo armBase = hardwareMap.servo.get("armBase");
        Servo rightFlip = hardwareMap.servo.get("rightFlip");
        Servo leftFlip = hardwareMap.servo.get("leftFlip");
        rightFlip.setPosition(0.5);
        leftFlip.setPosition(0);

        pixelDropper.setPosition(0.5);
        claw.setPosition(1.0);
        armBase.setPosition(0.55);
        armTop.setPosition(1.0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d backboardPose = new Pose2d();
        // build trajectories
        Pose2d startPose = new Pose2d(-39, 63, Math.toRadians(270));

        drive.setPoseEstimate(startPose);


        Trajectory purpleRightEntry = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-42, 39), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();

        Trajectory purpleRightBackdrop = drive.trajectoryBuilder(purpleRightEntry.end())
                .lineTo(new Vector2d(-42, 49))
                .splineToSplineHeading(new Pose2d(-39, 58, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(40,58))
                .splineTo(new Vector2d(51.5, 32), Math.toRadians(0))
                .build();

        Trajectory purpleCenterEntry = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-35, 30), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();
        Trajectory purpleCenterBackdrop = drive.trajectoryBuilder(purpleCenterEntry.end())
                .lineTo(new Vector2d(-45, 40))
                .splineToSplineHeading(new Pose2d(-39, 58, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(40,58))
                .splineTo(new Vector2d(51.5, 39), Math.toRadians(0))
                .build();

        Trajectory purpleLeftEntry = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-39, 35.5), Math.toRadians(0))
                .lineTo(new Vector2d(-30, 35.5))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();
        Trajectory purpleLeftBackdrop = drive.trajectoryBuilder(purpleLeftEntry.end())
                .lineTo(new Vector2d(-39, 37.5),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-39, 58, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(40,58))
                .splineTo(new Vector2d(51.5, 44), Math.toRadians(0))
                .build();

        // initialise vision (will loop in background)
        overallEOCVprocessor = new overallEOCVprocessor(left, right, false);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), overallEOCVprocessor);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        while(isStarted() == false){
            telemetry.addData("Identified", overallEOCVprocessor.getSelection());
            telemetry.addData("Left", overallEOCVprocessor.percentageLeft);
            telemetry.addData("Right", overallEOCVprocessor.percentageRight);
            telemetry.update();
        }
        // ends vision looping & gets final selection
        visionPortal.stopStreaming();
        Selected finalSelection = overallEOCVprocessor.getSelection();

        if(isStopRequested()) return;

        // purple pixel --> use vision to choose drop path
        if(finalSelection == Selected.NONE) {
            drive.followTrajectory(purpleLeftEntry);
            drive.followTrajectory(purpleLeftBackdrop);
            backboardPose = purpleLeftBackdrop.end();
        }
        else if (finalSelection == Selected.LEFT) {
            drive.followTrajectory(purpleCenterEntry);
            drive.followTrajectory(purpleCenterBackdrop);
            backboardPose = purpleCenterBackdrop.end();
        }
        else{
            // assumes left
            drive.followTrajectory(purpleRightEntry);
            drive.followTrajectory(purpleRightBackdrop);
            backboardPose = purpleRightBackdrop.end();
        }
        // drop on backdrop
        int target;
        if (partnerPlaced == true){
            // partner on board, pixel below paper blocks
            target = (int) (58 * COUNTS_PER_CM_slide) + slide.getCurrentPosition();
        } else{
            // partner not on board, pixel on paper blocks
            target = (int) (55 * COUNTS_PER_CM_slide) + slide.getCurrentPosition();
        }
        slide.setPower(0.4);
        while(slide.getCurrentPosition() < target && opModeIsActive()){slide.setPower(0.4);}
        slide.setPower(0.0005);
        armBase.setPosition(0.35);
        armTop.setPosition(0.37);
        sleep (500);
        claw.setPosition(0.05);
        sleep(400);

        if(parkingLocation == 1) {
            Trajectory park1A = drive.trajectoryBuilder(backboardPose)
                    .lineTo(new Vector2d(45, backboardPose.getY()))
                    .build();
            Trajectory park1B = drive.trajectoryBuilder(park1A.end())
                    .lineTo(new Vector2d(45, 15))
                    .build();
            drive.followTrajectory(park1A);
            drive.followTrajectory(park1B);

        }else if (parkingLocation == 2){
            Trajectory park2A = drive.trajectoryBuilder(backboardPose)
                    .lineTo(new Vector2d(47, backboardPose.getY()))
                    .build();
            Trajectory park2B = drive.trajectoryBuilder(park2A.end())
                    .lineTo(new Vector2d(47, 60))
                    .build();
            drive.followTrajectory(park2A);
            drive.followTrajectory(park2B);
        }
    }

}
