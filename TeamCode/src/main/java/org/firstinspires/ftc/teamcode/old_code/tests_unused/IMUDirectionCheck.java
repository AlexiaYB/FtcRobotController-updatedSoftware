package org.firstinspires.ftc.teamcode.old_code.tests_unused;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMUDirectionCheck", group = "Checks")
@Disabled
public class IMUDirectionCheck extends LinearOpMode {
    // declaring variables
    private ElapsedTime runtime = new ElapsedTime();
    // imu
    private IMU imu = null;

    @Override
    public void runOpMode() {
        // initialising variables
        // imu
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // indicating initialisation complete
        telemetry.addData("Status", "Initialized & Connected");
        telemetry.update();



        // when start pressed
        waitForStart();
        runtime.reset();

        // run opMode
        while (opModeIsActive()) {
            telemetry.addData("current heading: ", getHeading());
            telemetry.update();

        }
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}

