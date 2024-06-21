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
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;
import org.firstinspires.ftc.teamcode.overallEOCVprocessor.Selected;

@Autonomous(name = "autoBB", group = "Autonomous")
@Config
public class autoBB extends LinearOpMode {
    // adjustment constants!!!!!!
    // true if alliance partner has placed a yellow pixel on the board before us
    boolean partnerPlaced = true;
    // 0 means stay at board, 1 means to the pointy corner, 2 means to the square corner
    int parkingLocation = 1;


    private ElapsedTime visionTimer = new ElapsedTime();
    private overallEOCVprocessor overallEOCVprocessor;
    private VisionPortal visionPortal;
    public static Rect left = new Rect(0, 110, 135, 200);
    public static Rect right = new Rect(350, 90, 160, 170);
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
        Pose2d startPose = new Pose2d(15, 63, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory visionMove = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(22,63))
                .build();

        Trajectory purpleLeftEntry = drive.trajectoryBuilder(visionMove.end())
                .splineTo(new Vector2d(28, 39), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();

        Trajectory purpleLeftBackdrop = drive.trajectoryBuilder(purpleLeftEntry.end())
                .lineTo(new Vector2d(28, 48))
                .splineToSplineHeading(new Pose2d(49, 44, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory purpleCenterEntry = drive.trajectoryBuilder(visionMove.end())
                .splineTo(new Vector2d(13, 29), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();
        Trajectory purpleCenterBackdrop = drive.trajectoryBuilder(purpleCenterEntry.end())
                .lineTo(new Vector2d(13, 40))
                .splineToSplineHeading(new Pose2d(49, 39, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory purpleRightEntry = drive.trajectoryBuilder(visionMove.end())
                .splineTo(new Vector2d(8, 30), Math.toRadians(200))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();
        Trajectory purpleRightBackdrop = drive.trajectoryBuilder(purpleRightEntry.end())
                .lineTo(new Vector2d(15, 30),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(49, 32, Math.toRadians(0)), Math.toRadians(0))
                .build();

        // initialise vision (will loop in background)
        overallEOCVprocessor = new overallEOCVprocessor(left, right, false);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), overallEOCVprocessor);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        waitForStart();

        if(isStopRequested()) return;
        // move to detection point & do vision
        drive.followTrajectory(visionMove);
        visionTimer.reset();
        while (visionTimer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Identified", overallEOCVprocessor.getSelection());
            telemetry.addData("Left", overallEOCVprocessor.percentageLeft);
            telemetry.addData("Right", overallEOCVprocessor.percentageRight);
            telemetry.update();
        }
        visionPortal.stopStreaming();
        Selected finalSelection = overallEOCVprocessor.getSelection();

        // purple pixel --> use vision to choose drop path
        if(finalSelection == Selected.NONE) {
            drive.followTrajectory(purpleRightEntry);
            drive.followTrajectory(purpleRightBackdrop);
            backboardPose = purpleRightBackdrop.end();
        }
        else if (finalSelection == Selected.RIGHT) {
            drive.followTrajectory(purpleCenterEntry);
            drive.followTrajectory(purpleCenterBackdrop);
            backboardPose = purpleCenterBackdrop.end();
        }
        else{
            // assumes left
            drive.followTrajectory(purpleLeftEntry);
            drive.followTrajectory(purpleLeftBackdrop);
            backboardPose = purpleLeftBackdrop.end();
        }
        // drop on backdrop
//        int target;
//        if (partnerPlaced == true){
//            // partner on board, pixel below paper blocks
//            target = (int) (58 * COUNTS_PER_CM_slide) + slide.getCurrentPosition();
//        } else{
//            // partner not on board, pixel on paper blocks
//            target = (int) (55 * COUNTS_PER_CM_slide) + slide.getCurrentPosition();
//        }
//        slide.setPower(0.4);
//        while(slide.getCurrentPosition() < target && opModeIsActive()){slide.setPower(0.4);}
//        slide.setPower(0.0005);
//        armBase.setPosition(0.35);
//        armTop.setPosition(0.39);
//        sleep (500);
//        claw.setPosition(0.05);
//        sleep(400);

        if(parkingLocation == 1) {
            Trajectory park1A = drive.trajectoryBuilder(backboardPose)
                    .lineTo(new Vector2d(45, backboardPose.getY()))
                    .build();
            Trajectory park1B = drive.trajectoryBuilder(park1A.end())
                    .lineTo(new Vector2d(45, 7))
                    .build();
            drive.followTrajectory(park1A);
            drive.followTrajectory(park1B);

        }else if (parkingLocation == 2){
            Trajectory park2A = drive.trajectoryBuilder(backboardPose)
                    .lineTo(new Vector2d(45, backboardPose.getY()))
                    .build();
            Trajectory park2B = drive.trajectoryBuilder(park2A.end())
                    .lineTo(new Vector2d(45, 63))
                    .build();
            drive.followTrajectory(park2A);
            drive.followTrajectory(park2B);
        }
    }

}
