package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;
import org.firstinspires.ftc.teamcode.overallEOCVprocessor.Selected;

@Autonomous(name = "baseAutoRB", group = "Autonomous")
public class baseAutoRB extends LinearOpMode {
    private overallEOCVprocessor overallEOCVprocessor;
    private VisionPortal visionPortal;
    public static Rect left = new Rect(0, 55, 120, 170);
    public static Rect right = new Rect(310, 100, 185, 200);
    @Override
    public void runOpMode() {
        // initialise
        Servo pixelDropper = hardwareMap.servo.get("pixelDropper");
        pixelDropper.setPosition(0.5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // build trajectories
        Pose2d startPose = new Pose2d(15, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory purpleRightEntry = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(18, -39), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();

        Trajectory purpleRightBackdrop = drive.trajectoryBuilder(purpleRightEntry.end())
                .lineTo(new Vector2d(18, -54))
                .splineToSplineHeading(new Pose2d(45, -39, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory purpleCenterEntry = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(11, -30), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    pixelDropper.setPosition(1.0);
                })
                .build();

        Trajectory purpleLeftEntry = drive.trajectoryBuilder(startPose)
            .splineTo(new Vector2d(15, -37.5), Math.toRadians(180))
            .lineTo(new Vector2d(6, -37.5),
                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .addDisplacementMarker(() -> {
                pixelDropper.setPosition(1.0);
            })
            .build();
        // initialise vision (will loop in background)
        overallEOCVprocessor = new overallEOCVprocessor(left, right, true);
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
        }
        else if (finalSelection == Selected.LEFT) {
            drive.followTrajectory(purpleCenterEntry);
        }
        else{
            // assumes right
            drive.followTrajectory(purpleRightEntry);
            drive.followTrajectory(purpleRightBackdrop);
        }

    }
}
